#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PI_F 3.14159265358979323846f
#define DEG_TO_RAD_F (PI_F / 180.0f)
#define RAD_TO_DEG_F (180.0f / PI_F)

#define RATE_100_HZ 10
#define RATE_250_HZ 4
#define RATE_500_HZ 2
#define RATE_SUPERVISOR 50
#define RATE_DO_EXECUTE(RATE, STEP) ((STEP) % (RATE) == 0)

#define CRTP_NBR_OF_PORTS 16
#define CRTP_TX_QUEUE_SIZE 200
#define CRTP_RX_QUEUE_SIZE 16

#ifndef CONFIG_IMU_MADGWICK_QUATERNION
#define CONFIG_IMU_MADGWICK_QUATERNION 0
#endif

typedef struct { float x, y, z; } Vec3;
typedef struct { float qw, qx, qy, qz; } Quaternion;
typedef struct { float roll, pitch, yaw; } Euler;
typedef struct { float x, y, z; } Position;

typedef struct {
    Vec3 acc;               /* g */
    Vec3 gyro;              /* deg/s */
    Vec3 mag;               /* uT */
    float baroPressure;     /* hPa */
    float baroTemperature;  /* deg C */
    float batteryVoltage;   /* V */
} SensorData;

typedef struct {
    Position position;      /* m */
    Vec3 velocity;          /* m/s */
    Euler attitude;         /* deg */
    Quaternion quaternion;
    Vec3 acceleration;      /* g */
    bool flying;
} State;

typedef State StateEstimate;

typedef enum {
    AXIS_MODE_DISABLE = 0,
    AXIS_MODE_VELOCITY = 1,
    AXIS_MODE_ABS = 2
} AxisMode;

typedef struct {
    uint8_t x, y, z, roll, pitch, yaw;
} SetpointMode;

typedef struct {
    SetpointMode mode;
    Euler attitude;          /* deg */
    Euler attitudeRate;      /* deg/s */
    Vec3 velocity;           /* m/s */
    Position position;       /* m */
    Quaternion attitudeQuaternion;
    uint16_t thrust;
} Setpoint;

typedef enum {
    CONTROL_MODE_LEGACY = 0,
    CONTROL_MODE_FORCE_TORQUE = 1,
    CONTROL_MODE_FORCE = 2
} ControlMode;

typedef struct {
    int16_t roll, pitch, yaw;
    uint16_t thrust;
    ControlMode controlMode;
    float thrustSi;
    float torqueX, torqueY, torqueZ;
    float armLength;
    float thrustToTorque;
    float normalizedForces[4];
} Control;

typedef enum {
    COMMANDER_PRIORITY_DISABLE = 0,
    COMMANDER_PRIORITY_LOWEST = 1,
    COMMANDER_PRIORITY_HIGHLEVEL = 10
} CommanderPriority;

typedef struct {
    float roll, pitch, yaw;
    uint16_t thrust;
    uint8_t rollMode;   /* 0 = angle, 1 = rate */
    uint8_t pitchMode;  /* 0 = angle, 1 = rate */
    uint8_t yawMode;    /* 0 = angle, 1 = rate */
    bool altHold;
    bool posHold;
    bool plusMode;
    bool careFree;
} CrtpCommanderRpyt;

typedef struct {
    Setpoint setpoint;
    bool thrustLocked;
    bool posSetMode;
    bool modeSet;
} CommanderSetpoint;

typedef struct {
    float kp, ki, kd;
    float integral;
    float previousInput;
    float outLimit;
    float output;
    bool inited;
} PidObject;

typedef struct {
    uint8_t port;
    uint8_t channel;
    uint8_t data[32];
    uint8_t size;
} CRTPPacket;

typedef bool (*CRTPCallback)(const CRTPPacket *packet);

typedef struct {
    bool (*send)(const CRTPPacket *packet);
    bool (*receive)(CRTPPacket *packet);
    void (*reset)(void);
    bool (*isConnected)(void);
} CRTPLink;

typedef enum {
    SUPERVISOR_STATE_CALIBRATION = 0,
    SUPERVISOR_STATE_PREFLIGHT,
    SUPERVISOR_STATE_PREFLIGHT_PASSED,
    SUPERVISOR_STATE_ARMING,
    SUPERVISOR_STATE_READYTOFLY,
    SUPERVISOR_STATE_FLYING,
    SUPERVISOR_STATE_WARNING_LEVELOUT,
    SUPERVISOR_STATE_LANDED,
    SUPERVISOR_STATE_CRASHED,
    SUPERVISOR_STATE_TUMBLED,
    SUPERVISOR_STATE_ESTOP,
    SUPERVISOR_STATE_WDT
} SupervisorState;

typedef enum {
    SUPERVISOR_CB_WARNING          = 0x0001,
    SUPERVISOR_CB_TIMEOUT          = 0x0002,
    SUPERVISOR_CB_PREFLIGHT_TIMEOUT= 0x0004,
    SUPERVISOR_CB_LANDING_TIMEOUT  = 0x0008,
    SUPERVISOR_CB_CRTP_STOP        = 0x0010,
    SUPERVISOR_CB_PARAM_STOP       = 0x0020,
    SUPERVISOR_CB_WATCHDOG_STOP    = 0x0040,
    SUPERVISOR_CB_SPINUP_TIMEOUT   = 0x0080,
    SUPERVISOR_CB_CRASH            = 0x0100,
    SUPERVISOR_CB_FREEFALL         = 0x0200,
    SUPERVISOR_CB_MOTOR_FAULT      = 0x0400,
    SUPERVISOR_CB_DECK_FAULT       = 0x0800
} SupervisorConditionBits;

typedef struct {
    bool crashDetectionGsEnabled;
    float crashDetectionGs;
    float freeFallThreshold;
    float tiltThreshold;
    float invertedThreshold;
    uint32_t tiltTimeoutMs;
    uint32_t invertedTimeoutMs;
    bool tumbleCheckEnabled;
    uint32_t spinupTimeoutMs;
    uint32_t commanderWarningTimeoutMs;
    uint32_t commanderTimeoutTimeoutMs;
    uint32_t preflightTimeoutMs;
    uint32_t landingTimeoutMs;
    uint32_t emergencyWatchdogTimeoutMs;
    bool autoArmingEnabled;
    uint32_t rpmMin;
    uint32_t rpmMax;
    uint32_t rpmNotRespondingThresholdMs;
    uint32_t rpmNotRespondingTimeoutMs;
} SupervisorSafetyConfig;

typedef struct {
    Euler euler;
    Quaternion quaternion;
    Vec3 position;
    Vec3 velocity;
} StateEstimateLog;

typedef struct { float x, y, z; } Vec3Log;

typedef struct {
    float pressure;    /* hPa */
    float temperature; /* deg C */
} BaroLog;

typedef struct {
    uint16_t m1, m2, m3, m4;
} MotorLog;

typedef struct {
    Euler euler;
    Quaternion quaternion;
    float accZ;
    float accZWithoutGravity;
    bool calibrated;
} Sensfusion6Log;

typedef struct {
    SupervisorState state;
    bool isArmed;
    bool isCrashed;
    bool isTumbled;
    bool isFlying;
    bool canFly;
    bool canArm;
    bool autoArming;
    bool trajectoryFlying;
    bool trajectoryFinished;
    bool trajectoryDisabled;
    bool deckFault;
    bool isLocked;
    uint32_t conditionBits;
    uint32_t lastFlyingTick;
    uint32_t lastCommanderTick;
    uint32_t spinupStartTick;
    uint32_t emergencyStopLastNotificationTick;
    float accNorm;
} SupervisorLog;

typedef struct {
    bool motorPass[4];
    bool batteryPass;
    float batterySag;
    uint8_t motorTestCount;
} HealthLogData;

/* Numeric helpers */
int16_t saturateSignedInt16(int32_t value);
float capAngle(float angle);

/* Sensfusion6 */
void sensfusion6Init(void);
bool sensfusion6Test(void);
void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt);
void estimatedGravityDirection(float *gx, float *gy, float *gz);
void sensfusion6GetEulerRPY(float *roll, float *pitch, float *yaw);
void sensfusion6GetQuaternion(float *qw, float *qx, float *qy, float *qz);
float sensfusion6GetAccZ(float ax, float ay, float az);
float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az);
float invSqrt(float x);

/* Power distribution and battery */
void powerDistribution(const Control *control, int32_t motorValues[4]);
bool powerDistributionCap(int32_t motorValues[4], uint32_t maxAllowedThrust,
                          uint32_t idleThrust);
float batteryCompensation(float oldValue, float supply, float alpha);
uint16_t motorsCompensateBatteryVoltage(uint16_t thrust, float nominalVoltage,
                                        float actualVoltage);
uint16_t motorForceToPwm(float force);

/* PID and controller */
void pidInit(PidObject *pid, float kp, float ki, float kd, float outLimit);
float pidUpdate(PidObject *pid, float measured, float setpoint);
void pidReset(PidObject *pid);
void attitudeControllerInit(void);
void attitudeControllerResetAll(void);
void attitudeControllerResetRoll(void);
void attitudeControllerResetPitch(void);
void attitudeControllerResetYaw(void);
void controllerSetYawMaxDelta(float maxDelta);
void controllerSetPositionControlThrust(float thrust);
void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, Control *control, float dt);

/* Commander RPYT */
void crtpCommanderRpytDecodeSetpoint(const CrtpCommanderRpyt *rpyt,
                                     CommanderPriority activePriority,
                                     bool posSetMode,
                                     CommanderSetpoint *out);
void rotateYaw(float *x, float *y, float angleDeg);

/* Commander arbitration */
void commanderInit(void);
void commanderSetSetpoint(const Setpoint *setpoint, CommanderPriority priority,
                          uint32_t tick);
bool commanderGetSetpoint(Setpoint *setpoint);
void commanderRelaxPriority(void);
uint32_t commanderInactivityTime(uint32_t currentTick);
CommanderPriority commanderGetActivePriority(void);

/* Supervisor */
void supervisorInit(void);
void supervisorUpdate(uint32_t tick, uint32_t stabilizerStep);
bool supervisorCanFly(void);
bool supervisorCanArm(void);
bool supervisorIsArmed(void);
bool supervisorIsCrashed(void);
bool supervisorRequestArming(void);
bool supervisorRequestCrashRecovery(bool doRecovery);
bool supervisorIsFlyingCheck(uint32_t currentTick);
bool supervisorIsTumbledCheck(const SensorData *sensors, uint32_t currentTick);
bool supervisorIsPreflightTimeout(uint32_t currentTick);
bool supervisorIsLandingTimeout(uint32_t currentTick);
void supervisorUpdateAndPopulateConditions(uint32_t currentTick);
bool supervisorAreMotorsAllowedToRun(void);
uint16_t supervisorGetInfoBitfield(void);
bool supervisorIsRPMatArmingValid(void);
void supervisorSetSensorData(const SensorData *sensors);
void supervisorSetMotorRatios(const uint16_t ratios[4]);
void supervisorSetMotorRPMs(const uint16_t rpms[4]);
void supervisorConfigureSafety(const SupervisorSafetyConfig *config);
void supervisorOverrideSetpoint(Setpoint *setpoint);
bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick);
void supervisorNotifyCommanderSetpoint(uint32_t tick);
void supervisorNotifyEmergencyStopWatchdog(uint32_t tick);

/* Estimator */
void estimatorInit(void);
bool estimatorFifoPut(const SensorData *data);
bool estimatorFifoGet(SensorData *data);
bool estimatorPush(const SensorData *data);
bool estimatorPop(SensorData *data);
void estimatorComplementary(SensorData *sensors, State *state, uint32_t tick);

/* Stabilizer */
void stabilizerInit(void);
void stabilizerTask(uint32_t tick, bool canFly, bool motorsAllowed,
                    bool healthTestRequest);
void stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint, uint32_t tick);
uint32_t compressState(const State *state);
bool rateSupervisorValidate(uint32_t rate);

/* Health */
void startPropTest(void);
void startBatTest(void);
bool healthShallWeRunTest(void);
void healthRunTests(uint32_t tick);
bool evaluatePropTest(float value, float lowThreshold, float highThreshold,
                      uint8_t motorIndex);
void restartBatTest(void);
float variance(const float *samples, uint8_t n);

/* CRTP */
void crtpInit(void);
bool crtpSendPacket(const CRTPPacket *packet);
bool crtpSendPacketBlock(const CRTPPacket *packet);
bool crtpReceivePacket(CRTPPacket *packet);
bool crtpReceivePacketBlock(CRTPPacket *packet);
bool crtpReceivePacketWait(CRTPPacket *packet, uint32_t timeoutMs);
void crtpRxTask(void);
void crtpTxTask(void);
void crtpSetLink(CRTPLink *link);
void crtpReset(void);
bool crtpIsConnected(void);
uint16_t crtpGetFreeTxQueuePackets(void);
bool crtpRegisterPortCB(uint8_t port, CRTPCallback callback);
bool crtpCreateRxQueue(uint8_t port);
void crtpUpdateStats(void);
void crtpSetTick(uint32_t tick);

/* Logs */
extern StateEstimateLog stateEstimate;
extern Vec3Log gyro;
extern Vec3Log acc;
extern BaroLog baro;
extern MotorLog motor;
extern Sensfusion6Log sensfusion6Log;
extern SupervisorLog supervisorLog;
extern HealthLogData healthLog;

#ifdef __cplusplus
}
#endif

#endif /* GENERATED_CODE_H */

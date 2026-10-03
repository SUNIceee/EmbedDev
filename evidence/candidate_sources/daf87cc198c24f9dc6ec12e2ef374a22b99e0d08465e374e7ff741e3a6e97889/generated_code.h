#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Rate / scheduler macros
 * ---------------------------------------------------------------------- */
#define RATE_100_HZ       10
#define RATE_250_HZ       4
#define RATE_SUPERVISOR   10
#define RATE_DO_EXECUTE(rate, step) (((step) % (rate)) == 0)

/* -------------------------------------------------------------------------
 * Commander priority levels
 * ---------------------------------------------------------------------- */
#define COMMANDER_PRIORITY_DISABLE      0
#define COMMANDER_PRIORITY_LOWEST       1
#define COMMANDER_PRIORITY_CRTP         2
#define COMMANDER_PRIORITY_HIGHLEVEL    100

/* -------------------------------------------------------------------------
 * Supervisor condition bits
 * ---------------------------------------------------------------------- */
#define SUPERVISOR_CB_SPINUP_TIMEOUT      (1u << 0)
#define SUPERVISOR_CB_SETPOINT_WARNING    (1u << 1)
#define SUPERVISOR_CB_SETPOINT_TIMEOUT    (1u << 2)
#define SUPERVISOR_CB_PREFLIGHT_TIMEOUT   (1u << 3)
#define SUPERVISOR_CB_LANDING_TIMEOUT     (1u << 4)
#define SUPERVISOR_CB_CRTP_STOP           (1u << 5)
#define SUPERVISOR_CB_PARAM_STOP          (1u << 6)
#define SUPERVISOR_CB_WATCHDOG_STOP       (1u << 7)
#define SUPERVISOR_CB_TUMBLED             (1u << 8)
#define SUPERVISOR_CB_FREE_FALL           (1u << 9)
#define SUPERVISOR_CB_MOTOR_FAULT         (1u << 10)
#define SUPERVISOR_CB_DECK_FAULT          (1u << 11)
#define SUPERVISOR_CB_CRASH               (1u << 12)

/* -------------------------------------------------------------------------
 * CRTP queue capacities
 * ---------------------------------------------------------------------- */
#define CRTP_NBR_OF_PORTS            16
#define CRTP_RX_QUEUE_CAPACITY       16
#define CRTP_TX_QUEUE_CAPACITY       200

/* -------------------------------------------------------------------------
 * Sensfusion6 mode configuration. Define CONFIG_IMU_MADGWICK_QUATERNION to
 * 1 before including this header to select the Madgwick gradient mode.
 * ---------------------------------------------------------------------- */
#ifndef CONFIG_IMU_MADGWICK_QUATERNION
#define CONFIG_IMU_MADGWICK_QUATERNION 0
#endif

/* -------------------------------------------------------------------------
 * Basic vector / attitude / state / setpoint types
 * ---------------------------------------------------------------------- */
typedef struct {
    float x;
    float y;
    float z;
} Vector3_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float qw;
    float qx;
    float qy;
    float qz;
} Attitude_t;

typedef struct {
    Vector3_t gyro;      /* deg/s */
    Vector3_t acc;       /* g */
    Vector3_t mag;       /* uT */
    float baroPressure;  /* hPa */
    float baroTemp;      /* degC */
} SensorData_t;

typedef struct {
    Attitude_t attitude;
    Vector3_t position;
    Vector3_t velocity;
    Vector3_t acc;
} State_t;

typedef enum {
    SETPOINT_MODE_DISABLE = 0,
    SETPOINT_MODE_ABS = 1,
    SETPOINT_MODE_VELOCITY = 2,
    SETPOINT_MODE_QUAT = 3
} SetpointMode_t;

typedef struct {
    SetpointMode_t x;
    SetpointMode_t y;
    SetpointMode_t z;
    SetpointMode_t roll;
    SetpointMode_t pitch;
    SetpointMode_t yaw;
} SetpointModes_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float rateRoll;
    float ratePitch;
    float rateYaw;
} AttitudeSetpoint_t;

typedef struct {
    SetpointModes_t mode;
    AttitudeSetpoint_t attitude;
    Vector3_t velocity;
    Vector3_t position;
    float thrust;
} Setpoint_t;

typedef enum {
    CONTROL_MODE_LEGACY = 0,
    CONTROL_MODE_FORCE_TORQUE = 1,
    CONTROL_MODE_FORCE = 2
} ControlMode_t;

typedef struct {
    ControlMode_t controlMode;
    int32_t roll;
    int32_t pitch;
    int32_t yaw;
    int32_t thrust;
    float thrustSi;
    float torqueX;
    float torqueY;
    float torqueZ;
    float armLength;
    float thrustToTorque;
    float normalizedForces[4];
} Control_t;

/* -------------------------------------------------------------------------
 * PID types and declared PID objects
 * ---------------------------------------------------------------------- */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prevError;
    float output;
    float outLimit;
    float iLimit;
    bool reset;
} PidObject_t;

/* -------------------------------------------------------------------------
 * Estimator FIFO
 * ---------------------------------------------------------------------- */
typedef struct {
    float data[16];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} EstimatorFifo_t;

/* -------------------------------------------------------------------------
 * Supervisor safety configuration
 * ---------------------------------------------------------------------- */
typedef struct {
    bool autoArming;
    bool tumbleCheckEnabled;

    uint32_t spinupTimeoutStartTick;
    uint32_t spinupTimeoutPeriod;

    float crashDetectionGs;
    float freeFallThreshold;
    float tiltAccThreshold;
    float invertedAccThreshold;
    uint32_t tiltTimeoutMs;
    uint32_t invertedTimeoutMs;

    uint32_t setpointWarningTimeoutMs;
    uint32_t setpointTimeoutTimeoutMs;
    uint32_t preflightTimeoutMs;
    uint32_t landingTimeoutMs;

    int32_t rpmMin;
    int32_t rpmMax;
    int32_t rpmNotRespondingThreshold;
    uint32_t rpmNotRespondingTimeMs;

    int32_t maxAllowedThrust;
    int32_t idleThrust;
} SupervisorSafetyConfig_t;

typedef enum {
    SUPERVISOR_STATE_INITIAL = 0,
    SUPERVISOR_STATE_PRE_FL_CHECKS_PASSED,
    SUPERVISOR_STATE_ARMING,
    SUPERVISOR_STATE_READY_TO_FLY,
    SUPERVISOR_STATE_FLYING,
    SUPERVISOR_STATE_WARNING_LEVEL_OUT,
    SUPERVISOR_STATE_LANDED,
    SUPERVISOR_STATE_EXCEPT_FREE_FALL,
    SUPERVISOR_STATE_CRASHED,
    SUPERVISOR_STATE_TUMBLED,
    SUPERVISOR_STATE_EMERGENCY_STOP,
    SUPERVISOR_STATE_WATCHDOG_TIMEOUT,
    SUPERVISOR_STATE_MOTOR_FAULT,
    SUPERVISOR_STATE_PARAM_STOPPED,
    SUPERVISOR_STATE_CRTP_STOPPED
} SupervisorState_t;

/* -------------------------------------------------------------------------
 * Log objects
 * ---------------------------------------------------------------------- */
typedef struct {
    float roll;
    float pitch;
    float yaw;
    float qw;
    float qx;
    float qy;
    float qz;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
} StateEstimateLog_t;

typedef struct {
    float x;
    float y;
    float z;
} GyroLog_t;

typedef struct {
    float x;
    float y;
    float z;
} AccLog_t;

typedef struct {
    float pressure;
    float temperature;
} BaroLog_t;

typedef struct {
    int32_t m1;
    int32_t m2;
    int32_t m3;
    int32_t m4;
} MotorLog_t;

typedef struct {
    float qw;
    float qx;
    float qy;
    float qz;
    float ax;
    float ay;
    float az;
} Sensfusion6Log_t;

typedef struct {
    uint16_t infoBitfield;
    float accNorm;
} SupervisorLog_t;

typedef struct {
    uint32_t motorPass;
    bool batteryPass;
    float batterySag;
    uint32_t motorTestCount;
} HealthLog_t;

/* -------------------------------------------------------------------------
 * CRTP packet and link
 * ---------------------------------------------------------------------- */
typedef struct {
    uint8_t port;
    uint8_t channel;
    uint8_t data[32];
    uint8_t length;
} CRTPPacket_t;

typedef struct CRTPLink_t CRTPLink_t;
struct CRTPLink_t {
    bool (*send)(const CRTPPacket_t *packet);
    bool (*receive)(CRTPPacket_t *packet);
    void (*reset)(void);
    bool (*isConnected)(void);
};

/* -------------------------------------------------------------------------
 * Sensfusion6 globals
 * ---------------------------------------------------------------------- */
extern float qw;
extern float qx;
extern float qy;
extern float qz;
extern float integralFBx;
extern float integralFBy;
extern float integralFBz;
extern float baseZacc;
extern bool calibrated;
extern bool sensfusion6Initialized;
extern float twoKp;
extern float twoKi;

/* -------------------------------------------------------------------------
 * PID globals
 * ---------------------------------------------------------------------- */
extern PidObject_t pidRoll;
extern PidObject_t pidPitch;
extern PidObject_t pidYaw;
extern PidObject_t pidRollRate;
extern PidObject_t pidPitchRate;
extern PidObject_t pidYawRate;
extern float attitudeUpdateDt;
extern float yawMaxDelta;
extern float attitudeDesiredYaw;

/* -------------------------------------------------------------------------
 * Supervisor globals
 * ---------------------------------------------------------------------- */
extern SupervisorState_t supervisorState;
extern uint32_t supervisorConditionBits;
extern bool supervisorArmed;
extern bool supervisorCrashed;
extern bool supervisorTumbled;
extern bool supervisorFreeFalling;
extern bool supervisorIsLocked;
extern bool supervisorSeenFlight;
extern uint32_t supervisorRecentFlightTick;
extern uint32_t supervisorTick;
extern SensorData_t supervisorSensorData;
extern int32_t supervisorMotorRatios[4];
extern int32_t supervisorMotorRPMs[4];
extern SupervisorSafetyConfig_t supervisorSafetyConfig;

/* -------------------------------------------------------------------------
 * Estimator / commander / stabilizer globals
 * ---------------------------------------------------------------------- */
extern EstimatorFifo_t estimatorFifo;
extern SensorData_t estimatorLastSensor;

/* -------------------------------------------------------------------------
 * CRTP Commander configuration booleans
 * ---------------------------------------------------------------------- */
extern bool thrustLocked;
extern bool crtpCommanderPosSetMode;
extern bool crtpCommanderPosHoldMode;
extern bool crtpCommanderAltHoldMode;
extern bool crtpCommanderPlusMode;
extern bool crtpCommanderCarefreeMode;
extern bool crtpCommanderRateRollPitch;
extern bool crtpCommanderRateYaw;
extern bool crtpCommanderModeSet;

/* -------------------------------------------------------------------------
 * Log objects
 * ---------------------------------------------------------------------- */
extern StateEstimateLog_t stateEstimate;
extern GyroLog_t gyro;
extern AccLog_t acc;
extern BaroLog_t baro;
extern MotorLog_t motor;
extern Sensfusion6Log_t sensfusion6Log;
extern SupervisorLog_t supervisorLog;
extern HealthLog_t healthLog;

/* -------------------------------------------------------------------------
 * Numerical helpers
 * ---------------------------------------------------------------------- */
int16_t saturateSignedInt16(int32_t value);
float capAngle(float angle);
float invSqrt(float x);

/* -------------------------------------------------------------------------
 * Sensfusion6
 * ---------------------------------------------------------------------- */
void sensfusion6Init(void);
bool sensfusion6Test(void);
void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt);
void estimatedGravityDirection(float *gx, float *gy, float *gz);
void sensfusion6GetEulerRPY(float *roll, float *pitch, float *yaw);
void sensfusion6GetQuaternion(float *q0, float *q1, float *q2, float *q3);
float sensfusion6GetAccZ(float ax, float ay, float az);
float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az);

/* -------------------------------------------------------------------------
 * Power distribution and battery compensation
 * ---------------------------------------------------------------------- */
void powerDistributionInit(void);
void powerDistribution(const Control_t *control, int32_t *motorPwm);
bool powerDistributionCap(int32_t *motorPwm, int32_t maxAllowedThrust,
                          int32_t idleThrust);
uint16_t motorForceToPwm(float force);
float batteryCompensation(float oldThrust, float supplyVoltage);
uint16_t motorsCompensateBatteryVoltage(uint16_t thrust, float nominalVoltage,
                                        float actualVoltage);

/* -------------------------------------------------------------------------
 * PID / attitude controller
 * ---------------------------------------------------------------------- */
void pidInit(PidObject_t *pid, float kp, float ki, float kd,
             float outLimit, float iLimit);
float pidUpdate(PidObject_t *pid, float error, float dt);
void pidReset(PidObject_t *pid);

void attitudeControllerInit(void);
void attitudeControllerResetAll(const State_t *state);
void controllerPid(const State_t *state, const Setpoint_t *setpoint,
                   const SensorData_t *sensors, Control_t *control, float dt);
void positionControllerSetThrust(float thrust);
float positionControllerGetThrust(const State_t *state,
                                  const Setpoint_t *setpoint);

/* -------------------------------------------------------------------------
 * CRTP Commander RPYT
 * ---------------------------------------------------------------------- */
void crtpCommanderRpytDecodeSetpoint(Setpoint_t *setpoint, float roll,
                                      float pitch, float yaw,
                                      uint16_t rawThrust, int activePriority);
void rotateYaw(float *x, float *y, float yawDegrees);

/* -------------------------------------------------------------------------
 * Supervisor public API
 * ---------------------------------------------------------------------- */
void supervisorSetSensorData(const SensorData_t *sensors);
void supervisorSetMotorRatios(const int32_t motorRatios[4]);
void supervisorSetMotorRPMs(const int32_t motorRPMs[4]);
void supervisorConfigureSafety(const SupervisorSafetyConfig_t *config);
void supervisorSetTick(uint32_t currentTick);

void supervisorUpdate(uint32_t stabilizerStep);
bool supervisorCanFly(void);
bool supervisorCanArm(void);
bool supervisorIsArmed(void);
bool supervisorIsCrashed(void);
bool supervisorRequestArming(void);
bool supervisorRequestCrashRecovery(bool doRecovery);
bool supervisorIsFlyingCheck(uint32_t currentTick);
bool supervisorIsTumbledCheck(void);
bool supervisorIsPreflightTimeout(void);
bool supervisorIsLandingTimeout(void);
void updateAndPopulateConditions(void);
bool supervisorIsRPMatArmingValid(void);
void supervisorOverrideSetpoint(Setpoint_t *setpoint, const State_t *state);
bool supervisorAreMotorsAllowedToRun(void);
uint16_t supervisorGetInfoBitfield(void);
bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick);

/* -------------------------------------------------------------------------
 * Estimator
 * ---------------------------------------------------------------------- */
void estimatorInit(void);
bool estimatorFifoPush(float value);
bool estimatorFifoPop(float *value);
void estimatorComplementary(State_t *state, SensorData_t *sensors,
                            uint32_t step);

/* -------------------------------------------------------------------------
 * Commander arbitration
 * ---------------------------------------------------------------------- */
void commanderInit(void);
bool commanderSetSetpoint(const Setpoint_t *setpoint, int priority);
void commanderGetSetpoint(Setpoint_t *setpoint, uint32_t currentTick);
int commanderRelaxPriority(void);
int commanderInactivityTime(uint32_t currentTick);
int commanderGetActivePriority(void);

/* -------------------------------------------------------------------------
 * Stabilizer
 * ---------------------------------------------------------------------- */
void sensorsInit(void);
void stateEstimatorInit(void);
void controllerInit(void);
void motorsInit(void);
void collisionAvoidanceInit(void);

void stabilizerInit(void);
void stabilizerTask(uint32_t stabilizerStep);
bool stabilizerSubmitHighLevelSetpoint(const Setpoint_t *setpoint);

void setMotorRatios(const int32_t motorRatios[4]);
void sensorsWaitDataReady(void);
void sensorsAcquire(SensorData_t *sensors, uint32_t tick);
void commanderGetSetpointFromStabilizer(Setpoint_t *setpoint, uint32_t tick);
void collisionAvoidanceUpdateSetpoint(Setpoint_t *setpoint, const State_t *state);

/* -------------------------------------------------------------------------
 * Compress state and frequency supervisor
 * ---------------------------------------------------------------------- */
uint32_t compressState(const State_t *state);
bool rateSupervisorValidate(uint32_t rate);

/* -------------------------------------------------------------------------
 * Health
 * ---------------------------------------------------------------------- */
void startPropTest(void);
void startBatTest(void);
bool healthShallWeRunTest(void);
void healthRunTests(uint32_t tick);
bool evaluatePropTest(void);
void restartBatTest(void);

/* -------------------------------------------------------------------------
 * CRTP transport
 * ---------------------------------------------------------------------- */
void crtpInit(void);
bool crtpSendPacket(const CRTPPacket_t *packet);
bool crtpSendPacketBlock(const CRTPPacket_t *packet);
bool crtpReceivePacket(CRTPPacket_t *packet);
bool crtpReceivePacketBlock(CRTPPacket_t *packet, uint32_t timeoutMs);
bool crtpReceivePacketWait(CRTPPacket_t *packet);
void crtpSetLink(CRTPLink_t *link);
void crtpReset(void);
bool crtpIsConnected(void);
uint16_t crtpGetFreeTxQueuePackets(void);
bool crtpRegisterPortCB(uint8_t port, void (*callback)(const CRTPPacket_t *packet));
void crtpUpdateStats(void);
void crtpRxTask(void);
void crtpTxTask(void);

#define SendPacket          crtpSendPacket
#define SendPacketBlock     crtpSendPacketBlock
#define ReceivePacket       crtpReceivePacket
#define ReceivePacketBlock  crtpReceivePacketBlock
#define ReceivePacketWait   crtpReceivePacketWait
#define SetLink             crtpSetLink
#define Reset               crtpReset
#define IsConnected         crtpIsConnected
#define GetFreeTxQueuePackets crtpGetFreeTxQueuePackets
#define RegisterPortCB      crtpRegisterPortCB
#define updateStats         crtpUpdateStats

/* -------------------------------------------------------------------------
 * Deck discovery
 * ---------------------------------------------------------------------- */
int deckDiscovery(uint8_t *buffer, int capacity);

/* -------------------------------------------------------------------------
 * Inline clamp helpers
 * ---------------------------------------------------------------------- */
static inline int32_t clampInt32(int32_t value, int32_t min, int32_t max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static inline float clampFloat(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

#ifdef __cplusplus
}
#endif

#endif /* GENERATED_CODE_H */

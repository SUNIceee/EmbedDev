// ==========================================
// HEADER: generated_code.h
// ==========================================
#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdint.h>
#include <stdbool.h>

// Macros
#define CRTP_NBR_OF_PORTS 16
#define COMMANDER_PRIORITY_DISABLE 0
#define COMMANDER_PRIORITY_LOWEST 1
#define COMMANDER_PRIORITY_HIGHLEVEL 3

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Structures
typedef struct {
    float x;
    float y;
    float z;
} vector3_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} attitude_t;

typedef struct {
    attitude_t attitude;
    attitude_t attitudeRate;
    uint16_t thrust;
    struct {
        uint8_t x;
        uint8_t y;
        uint8_t z;
        uint8_t yaw;
    } mode;
    vector3_t position;
    vector3_t velocity;
} setpoint_t;

typedef struct {
    attitude_t attitude;
    vector3_t position;
    vector3_t velocity;
} state_t;

typedef struct {
    vector3_t gyro;
    vector3_t acc;
} sensor_data_t;

typedef struct {
    uint8_t size;
    uint8_t header;
    uint8_t data[30];
} crtp_packet_t;

typedef void (*crtp_callback_t)(crtp_packet_t*);

typedef struct {
    bool (*send)(crtp_packet_t*);
    void (*reset)(void);
} crtp_link_t;

typedef enum {
    CONTROL_MODE_LEGACY = 0,
    CONTROL_MODE_FORCE_TORQUE,
    CONTROL_MODE_FORCE
} control_mode_t;

typedef struct {
    control_mode_t controlMode;
    int32_t roll;
    int32_t pitch;
    int32_t yaw;
    int32_t thrust;
    float thrustSi;
    float torqueX;
    float torqueY;
    float torqueZ;
    float normalizedForces[4];
} control_t;

typedef struct {
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
} motor_ratios_t;

typedef enum {
    MEASUREMENT_IMU = 0,
    MEASUREMENT_BARO,
    MEASUREMENT_MAG,
    MEASUREMENT_COUNT
} measurement_type_t;

typedef struct {
    measurement_type_t type;
    union {
        struct {
            float ax, ay, az;
            float gx, gy, gz;
        } imu;
        struct {
            float pressure;
            float temp;
        } baro;
        struct {
            float mx, my, mz;
        } mag;
    } data;
} measurement_t;

typedef struct {
    int16_t pos_x;
    int16_t pos_y;
    int16_t pos_z;
    int16_t vel_x;
    int16_t vel_y;
    int16_t vel_z;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    uint32_t quat;
} compressed_state_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float iLimit;
    float integrand;
    float prevError;
    float lastDeriv;
} PID_t;

// Log structures
typedef struct {
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float ax;
    float ay;
    float az;
    float roll;
    float pitch;
    float yaw;
    float q0;
    float q1;
    float q2;
    float q3;
} log_state_estimate_t;

typedef struct {
    float x;
    float y;
    float z;
} log_gyro_t;

typedef struct {
    float x;
    float y;
    float z;
} log_acc_t;

typedef struct {
    float asl;
    float temp;
    float pressure;
} log_baro_t;

typedef struct {
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
} log_motor_t;

typedef struct {
    float qw;
    float qx;
    float qy;
    float qz;
    float roll;
    float pitch;
    float yaw;
    bool isCalibrated;
} log_sensfusion6_t;

typedef struct {
    uint32_t info;
    float accNorm;
    uint8_t state;
    bool isArmed;
    bool isCrashed;
} log_supervisor_t;

typedef struct {
    uint8_t motorPass;
    bool batteryPass;
    float batterySag;
    uint32_t motorTestCount;
} log_health_t;

typedef struct {
    uint8_t i2cAddr;
    uint8_t onewireROM[8];
} deck_info_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    uint16_t thrust;
} crtp_rpyt_payload_t;

// Global Log instances
extern log_state_estimate_t stateEstimate;
extern log_gyro_t gyro;
extern log_acc_t acc;
extern log_baro_t baro;
extern log_motor_t motor;
extern log_sensfusion6_t sensfusion6Log;
extern log_supervisor_t supervisorLog;
extern log_health_t healthLog;

// Global variables for configuration and state testing
extern float qw, qx, qy, qz;
extern float integralFBx, integralFBy, integralFBz;
extern float twoKp, twoKi;
extern float baseZacc;
extern bool calibrated;
extern bool config_imu_madgwick;
extern PID_t pidRoll, pidPitch, pidYaw, pidRollRate, pidPitchRate, pidYawRate;
extern float yawMaxDelta;
extern bool thrustLocked;
extern bool altHoldMode;
extern bool posHoldMode;
extern bool posSetMode;
extern bool plusMode;
extern bool carefreeMode;
extern bool carefreeError;
extern uint8_t activePriority;
extern float positionControllerOutputThrust;
extern float batteryVoltage;
extern float batteryNominalVoltage;
extern uint32_t currentTick;
extern bool isArmed;
extern bool isCrashed;
extern bool isTumbled;
extern bool isFreeFalling;
extern bool isLocked;
extern bool trajectoryFlying;
extern bool trajectoryFinished;
extern bool trajectoryDisabled;
extern bool deckFault;
extern bool configAutoArming;
extern uint32_t recentFlightTick;
extern bool hasFlown;
extern int32_t motorRatios[4];
extern int32_t motorRPMs[4];
extern float crashDetectionGs;
extern float freeFallThreshold;
extern float tiltThreshold;
extern float invertThreshold;
extern bool tumbleCheckEnabled;
extern uint32_t spinupStartTick;
extern uint32_t spinupTimeoutDuration;
extern uint32_t lastWatchdogNotificationTick;
extern uint32_t lastCommanderSetpointTick;
extern uint32_t supervisorConditionBits;

// Functions
int16_t saturateSignedInt16(int32_t value);
float capAngle(float angle);
float invSqrt(float x);

void sensfusion6Init(void);
bool sensfusion6Test(void);
void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void estimatedGravityDirection(float* gx, float* gy, float* gz);
void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw);
void sensfusion6GetQuaternion(float* q);
float sensfusion6GetAccZ(float ax, float ay, float az);
float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az);

uint16_t motorForceToPwm(float force);
bool powerDistributionCap(int32_t* m1, int32_t* m2, int32_t* m3, int32_t* m4, int32_t maxAllowedThrust, int32_t idleThrust);
float batteryCompensation(float old, float supply, float alpha);
uint16_t motorsCompensateBatteryVoltage(uint16_t thrust, float nominal, float actual);

void pidInit(PID_t* pid, float kp, float ki, float kd, float iLimit);
float pidUpdate(PID_t* pid, float error, float dt);
void attitudeControllerInit(void);
void attitudeControllerResetAll(float currentRoll, float currentPitch, float currentYaw);
void attitudeControllerResetAxis(int axis);
void controllerPid(const sensor_data_t* sensors, const setpoint_t* setpoint, const state_t* state, control_t* control, float dt);

void crtpCommanderRpytDecodeSetpoint(setpoint_t* setpoint, const crtp_rpyt_payload_t* payload);

bool supervisorCanFly(void);
bool supervisorCanArm(void);
bool supervisorIsArmed(void);
bool supervisorIsCrashed(void);
bool supervisorRequestArming(bool arm);
bool supervisorRequestCrashRecovery(bool doRecovery);
bool isFlyingCheck(int32_t idleThrust);
bool isTumbledCheck(float ax, float ay, float az);
void supervisorCheckSpinup(void);
bool checkEmergencyStopWatchdog(uint32_t cur, uint32_t last);
void supervisorCheckSetpointAge(void);
bool supervisorIsPreflightTimeout(uint32_t duration);
bool supervisorIsLandingTimeout(uint32_t duration);
bool isRPMatArmingValid(void);
void supervisorSetSensorData(float ax, float ay, float az);
void supervisorSetMotorRatios(const int32_t* ratios);
void supervisorSetMotorRPMs(const int32_t* rpms);
void supervisorConfigureSafety(float crashGs, float ffThresh, float tiltThresh, float invThresh, bool tumbleCheck);
void supervisorOverrideSetpoint(setpoint_t* setpoint);
bool supervisorAreMotorsAllowedToRun(void);
uint32_t supervisorGetInfoBitfield(void);
void supervisorUpdate(uint32_t step);

bool estimatorEnqueue(const measurement_t* m);
bool estimatorDequeue(measurement_t* m);
void estimatorComplementary(uint32_t step);

bool commanderSetSetpoint(const setpoint_t* setpoint, uint8_t priority);
void commanderRelaxPriority(void);
uint32_t commanderGetInactivityTime(uint32_t curTick);
uint8_t commanderGetActivePriority(void);

void sensorsInit(void);
void stateEstimatorInit(void);
void controllerInit(void);
void powerDistributionInit(void);
void motorsInit(void);
void collisionAvoidanceInit(void);
void stabilizerInit(void);
void stabilizerSubmitHighLevelSetpoint(const setpoint_t* setpoint);
void stabilizerTaskStep(uint32_t step);
uint32_t quatcompress(float qw, float qx, float qy, float qz);
void compressState(const state_t* state, const sensor_data_t* sensors, compressed_state_t* compressed);
bool rateSupervisorValidate(uint32_t hz);

void startPropTest(void);
void startBatTest(void);
bool healthShallWeRunTest(void);
void healthRunTests(uint32_t step);
void restartBatTest(void);

void crtpInit(void);
bool crtpCreatePortQueue(int port);
bool crtpRegisterPortCB(int port, crtp_callback_t cb);
bool crtpSendPacket(crtp_packet_t* p);
bool crtpSendPacketBlock(crtp_packet_t* p);
int crtpGetFreeTxQueuePackets(void);
void crtpSetLink(crtp_link_t* link);
void crtpReset(void);
bool crtpIsConnected(void);
void crtpUpdateStats(uint32_t tick);
bool crtpReceivePacket(int port, crtp_packet_t* p);
bool crtpReceivePacketBlock(int port, crtp_packet_t* p);
bool crtpReceivePacketWait(int port, crtp_packet_t* p, uint32_t timeout);

void simulateCrtpRx(crtp_packet_t* p);
void simulateCrtpTxStep(uint32_t step);

int deckDiscovery(const uint8_t* i2cAddresses, int i2cCount, const uint8_t* owROMs, int owCount, deck_info_t* discovered, int capacity);

bool sensorsWaitDataReady(void);
void sensorsAcquire(sensor_data_t* sensors);

#endif // GENERATED_CODE_H

/* DOCSTRING: Supervisor safety state machine, arming, crash/tumble detection, and condition bits. */
#include "6_generated_code.h"
#include "crazyflie_internal.h"

#include <math.h>
#include <string.h>

SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0u;

static bool gArmed = false;
static bool gCrashed = false;
static bool gTumbled = false;
static bool gIsFlying = false;
static bool gFreeFalling = false;
static bool gAutoArming = false;
static bool gSupervisorInitialized = false;

static uint32_t gSpinupStartTick = 0u;
static uint32_t gSpinupTimeoutDurationMs = 0u;

static SensorData gSensorData;
static uint32_t gMotorRatios[4] = {0u, 0u, 0u, 0u};
static int32_t gMotorRPMs[4] = {0, 0, 0, 0};
static uint32_t gIdleThrust = 0u;

static float gCrashDetectionGs = 0.0f;
static float gFreeFallThreshold = 0.0f;
static float gAcceptedTiltAccZ = 0.0f;
static float gAcceptedUpsideDownAccZ = 0.0f;
static uint32_t gMaxTiltTime = 0u;
static uint32_t gMaxUpsideDownTime = 0u;
static bool gTumbleCheckEnabled = false;

static bool gTrajectoryFlying = false;
static bool gTrajectoryFinished = false;
static bool gTrajectoryDisabled = false;
static bool gDeckFault = false;

static bool sTumbleTimerRunning = false;
static uint32_t sTumbleTimerStart = 0u;

static bool sFlightSeen = false;
static uint32_t sLastFlightTick = 0u;

static bool sNotRespondingActive = false;
static uint32_t sNotRespondingStart = 0u;

static SupervisorState sPrevState = supervisorStateLocked;

void supervisorInit(void)
{
    if (gSupervisorInitialized) {
        return;
    }

    supervisorState = supervisorStateLocked;
    supervisorConditionBits = 0u;

    gArmed = false;
    gCrashed = false;
    gTumbled = false;
    gIsFlying = false;
    gFreeFalling = false;
    gAutoArming = false;
    gSpinupStartTick = 0u;
    gSpinupTimeoutDurationMs = 0u;

    memset(&gSensorData, 0, sizeof(gSensorData));
    memset(gMotorRatios, 0, sizeof(gMotorRatios));
    memset(gMotorRPMs, 0, sizeof(gMotorRPMs));
    gIdleThrust = 0u;

    gCrashDetectionGs = 0.0f;
    gFreeFallThreshold = 0.0f;
    gAcceptedTiltAccZ = 0.0f;
    gAcceptedUpsideDownAccZ = 0.0f;
    gMaxTiltTime = 0u;
    gMaxUpsideDownTime = 0u;
    gTumbleCheckEnabled = false;

    gTrajectoryFlying = false;
    gTrajectoryFinished = false;
    gTrajectoryDisabled = false;
    gDeckFault = false;

    sTumbleTimerRunning = false;
    sTumbleTimerStart = 0u;
    sFlightSeen = false;
    sLastFlightTick = 0u;
    sNotRespondingActive = false;
    sNotRespondingStart = 0u;
    sPrevState = supervisorStateLocked;

    gSupervisorInitialized = true;
}

bool supervisorCanFly(void)
{
    return supervisorState == supervisorStateReadyToFly ||
           supervisorState == supervisorStateFlying ||
           supervisorState == supervisorStateWarningLevelOut ||
           supervisorState == supervisorStateLanded;
}

bool supervisorCanArm(void)
{
    return supervisorState == supervisorStatePreFlChecksPassed;
}

bool supervisorIsArmed(void)
{
    return gArmed;
}

bool supervisorIsCrashed(void)
{
    return gCrashed;
}

bool supervisorAreMotorsAllowedToRun(void)
{
    return supervisorState == supervisorStateArming ||
           supervisorState == supervisorStateReadyToFly ||
           supervisorState == supervisorStateFlying ||
           supervisorState == supervisorStateWarningLevelOut ||
           supervisorState == supervisorStateLanded;
}

uint16_t supervisorGetInfoBitfield(void)
{
    uint16_t info = 0u;

    if (supervisorCanArm()) {
        info |= (1u << 0);
    }
    if (gArmed) {
        info |= (1u << 1);
    }
    if (gAutoArming) {
        info |= (1u << 2);
    }
    if (supervisorCanFly()) {
        info |= (1u << 3);
    }
    if (gIsFlying) {
        info |= (1u << 4);
    }
    if (gTumbled) {
        info |= (1u << 5);
    }
    if (supervisorState == supervisorStateLocked) {
        info |= (1u << 6);
    }
    if (gCrashed) {
        info |= (1u << 7);
    }
    if (gTrajectoryFlying) {
        info |= (1u << 8);
    }
    if (gTrajectoryFinished) {
        info |= (1u << 9);
    }
    if (gTrajectoryDisabled) {
        info |= (1u << 10);
    }
    if (gDeckFault) {
        info |= (1u << 11);
    }

    return info;
}

bool supervisorRequestArming(bool doArm)
{
    if (!doArm) {
        if (gArmed) {
            gArmed = false;
            supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
        }
        return false;
    }

    if (supervisorState == supervisorStateArming && gArmed) {
        return true;
    }

    if (!supervisorCanArm()) {
        return false;
    }

    gArmed = true;
    supervisorState = supervisorStateArming;
    gSpinupStartTick = g_systemTickMs;
    supervisorConditionBits |= SUPERVISOR_CB_ARMED;
    return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery)
{
    if (gTumbled) {
        return false;
    }

    if (doRecovery) {
        gCrashed = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
        return true;
    }

    gCrashed = true;
    supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    return true;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick)
{
    if (motorRatios == NULL) {
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        if (motorRatios[i] > idleThrust) {
            sFlightSeen = true;
            sLastFlightTick = currentTick;
            break;
        }
    }

    if (!sFlightSeen) {
        return false;
    }

    uint32_t elapsed = currentTick - sLastFlightTick;
    return elapsed < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling)
{
    if (isFreeFalling != NULL) {
        *isFreeFalling = false;
    }

    if (!tumbleCheckEnabled) {
        sTumbleTimerRunning = false;
        gTumbled = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
        return false;
    }

    float norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
    if (crashDetectionGs > 0.0f &&
        fabsf(norm - 1.0f) > crashDetectionGs) {
        gCrashed = true;
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    }

    if (fabsf(accX) < freeFallThreshold &&
        fabsf(accY) < freeFallThreshold &&
        fabsf(accZ) < freeFallThreshold) {
        if (isFreeFalling != NULL) {
            *isFreeFalling = true;
        }
        gFreeFalling = true;
        supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
        sTumbleTimerRunning = false;
        gTumbled = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
        return false;
    }

    gFreeFalling = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;

    uint32_t timeout = 0u;
    if (accZ < acceptedUpsideDownAccZ) {
        timeout = maxUpsideDownTime;
    } else if (accZ < acceptedTiltAccZ) {
        timeout = maxTiltTime;
    } else {
        sTumbleTimerRunning = false;
        gTumbled = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
        return false;
    }

    if (!sTumbleTimerRunning) {
        sTumbleTimerRunning = true;
        sTumbleTimerStart = currentTick;
    }

    if ((currentTick - sTumbleTimerStart) >= timeout) {
        gTumbled = true;
        supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
        return true;
    }

    gTumbled = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
    return false;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick)
{
    if (lastNotificationTick == 0u) {
        return true;
    }

    return (currentTick - lastNotificationTick) <=
           DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration)
{
    if (state != supervisorStateReadyToFly || latestArmingTick == 0u) {
        return false;
    }

    return (currentTick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTick,
                                uint32_t landingTimeoutDuration)
{
    if (latestLandingTick == 0u) {
        return false;
    }

    return (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed)
{
    if (gArmed) {
        supervisorConditionBits |= SUPERVISOR_CB_ARMED;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    }

    if (gIsFlying) {
        supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;
    }

    if (gTumbled) {
        supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
    }

    if (gCrashed) {
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    }

    if (gFreeFalling) {
        supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;
    }

    if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
    }

    supervisorLog.info = supervisorGetInfoBitfield();
    supervisorLog.accNorm =
        sqrtf(gSensorData.acc.x * gSensorData.acc.x +
              gSensorData.acc.y * gSensorData.acc.y +
              gSensorData.acc.z * gSensorData.acc.z);

    return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t supervisorConditionBits_,
                                SupervisorState state)
{
    if (setpoint == NULL) {
        return;
    }

    if (state == supervisorStateArming ||
        state == supervisorStateReadyToFly ||
        state == supervisorStateFlying ||
        state == supervisorStateLanded) {
        return;
    }

    if (state == supervisorStateWarningLevelOut) {
        StabilizationMode zMode = setpoint->mode.z;
        uint16_t zThrust = setpoint->thrust;
        float zPosition = setpoint->position.z;
        float zVelocity = setpoint->velocity.z;

        memset(setpoint, 0, sizeof(*setpoint));

        setpoint->mode.roll = modeAbs;
        setpoint->mode.pitch = modeAbs;
        setpoint->mode.yaw = modeVelocity;

        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->attitudeRate.yaw = 0.0f;

        setpoint->mode.z = zMode;
        setpoint->thrust = zThrust;
        setpoint->position.z = zPosition;
        setpoint->velocity.z = zVelocity;
        return;
    }

    memset(setpoint, 0, sizeof(*setpoint));
    if (supervisorConditionBits_ & (SUPERVISOR_CB_EMERGENCY_STOP |
                                    SUPERVISOR_CB_MOTORS_NOT_RESPONDING)) {
        setpoint->thrust = 0;
    }
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax)
{
    if (motorRPMs == NULL) {
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) {
            return false;
        }
    }

    return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly,
                           uint32_t currentTick)
{
    if (!canFly || motorRPMs == NULL) {
        sNotRespondingActive = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return false;
    }

    bool below = false;
    for (int i = 0; i < 4; ++i) {
        if (motorRPMs[i] < rpmThreshold) {
            below = true;
            break;
        }
    }

    if (!below) {
        sNotRespondingActive = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return false;
    }

    if (!sNotRespondingActive) {
        sNotRespondingActive = true;
        sNotRespondingStart = currentTick;
    }

    if ((currentTick - sNotRespondingStart) >= rpmCheckDurationMs) {
        supervisorConditionBits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return true;
    }

    supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    return false;
}

void supervisorSetSensorData(const SensorData *sensors)
{
    if (sensors == NULL) {
        return;
    }

    gSensorData = *sensors;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust)
{
    if (motorRatios == NULL) {
        return;
    }

    for (int i = 0; i < 4; ++i) {
        gMotorRatios[i] = motorRatios[i];
    }
    gIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4])
{
    if (motorRPMs == NULL) {
        return;
    }

    for (int i = 0; i < 4; ++i) {
        gMotorRPMs[i] = motorRPMs[i];
    }
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled)
{
    gCrashDetectionGs = crashDetectionGs;
    gFreeFallThreshold = freeFallThreshold;
    gAcceptedTiltAccZ = acceptedTiltAccZ;
    gAcceptedUpsideDownAccZ = acceptedUpsideDownAccZ;
    gMaxTiltTime = maxTiltTime;
    gMaxUpsideDownTime = maxUpsideDownTime;
    gTumbleCheckEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs)
{
    gAutoArming = autoArming;
    gSpinupTimeoutDurationMs = spinupTimeoutDurationMs;
    gSpinupStartTick = 0u;
}

void supervisorUpdate(uint32_t stabilizerStep)
{
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) {
        return;
    }

    g_systemTickMs = stabilizerStep;

    bool freeFall = false;
    gTumbled = isTumbledCheck(gSensorData.acc.x, gSensorData.acc.y,
                              gSensorData.acc.z,
                              gCrashDetectionGs, gFreeFallThreshold,
                              gAcceptedTiltAccZ, gAcceptedUpsideDownAccZ,
                              gMaxTiltTime, gMaxUpsideDownTime,
                              gTumbleCheckEnabled, g_systemTickMs,
                              &freeFall);
    gFreeFalling = freeFall;
    gIsFlying = isFlyingCheck(gMotorRatios, gIdleThrust, g_systemTickMs);

    uint32_t commanderAge = commanderGetInactivityTime();
    if (commanderAge > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    }

    if (commanderAge > COMMANDER_WDT_TIMEOUT_STABILIZE) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    }

    if (gAutoArming &&
        sPrevState != supervisorStatePreFlChecksPassed &&
        supervisorState == supervisorStatePreFlChecksPassed) {
        supervisorRequestArming(true);
    }

    if (gArmed &&
        !supervisorAreMotorsAllowedToRun() &&
        supervisorState != supervisorStateArming) {
        gArmed = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    }

    if (supervisorState == supervisorStateArming &&
        gSpinupStartTick != 0u &&
        gSpinupTimeoutDurationMs != 0u &&
        (g_systemTickMs - gSpinupStartTick) >= gSpinupTimeoutDurationMs) {
        supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
    } else if (supervisorState != supervisorStateArming) {
        gSpinupStartTick = 0u;
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    }

    updateAndPopulateConditions(false, false, false);
    sPrevState = supervisorState;
}

/* DOCSTRING: Bounded 16-slot estimator FIFO and commander priority arbitration. */
#include "6_generated_code.h"
#include "crazyflie_internal.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

State g_estimatedState;
Setpoint g_activeSetpoint;

#define ESTIMATOR_FIFO_CAPACITY 16U

static EstimatorMeasurement s_estimatorFifo[ESTIMATOR_FIFO_CAPACITY];
static uint8_t s_estimatorHead = 0u;
static uint8_t s_estimatorTail = 0u;
static uint8_t s_estimatorCount = 0u;

static EstimatorMeasurement s_lastGyro;
static EstimatorMeasurement s_lastAcc;
static EstimatorMeasurement s_lastBaro;
static EstimatorMeasurement s_lastTof;
static bool s_hasGyro = false;
static bool s_hasAcc = false;
static bool s_hasBaro = false;
static bool s_hasTof = false;

static int g_activePriority = COMMANDER_PRIORITY_LOWEST;
static uint32_t g_commanderLastUpdateTick = 0u;
static bool g_highLevelSetpointActive = false;

bool estimatorEnqueue(const EstimatorMeasurement *measurement)
{
    if (measurement == NULL) {
        return false;
    }

    if (s_estimatorCount >= ESTIMATOR_FIFO_CAPACITY) {
        return false;
    }

    s_estimatorFifo[s_estimatorTail] = *measurement;
    s_estimatorTail = (uint8_t)((s_estimatorTail + 1u) % ESTIMATOR_FIFO_CAPACITY);
    s_estimatorCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement)
{
    if (measurement == NULL) {
        return false;
    }

    if (s_estimatorCount == 0u) {
        return false;
    }

    *measurement = s_estimatorFifo[s_estimatorHead];
    s_estimatorHead = (uint8_t)((s_estimatorHead + 1u) % ESTIMATOR_FIFO_CAPACITY);
    s_estimatorCount--;
    return true;
}

static void updateEstimatorLogs(void)
{
    stateEstimate.roll = g_estimatedState.attitude.roll;
    stateEstimate.pitch = g_estimatedState.attitude.pitch;
    stateEstimate.yaw = g_estimatedState.attitude.yaw;

    stateEstimate.qx = g_estimatedState.attitudeQuaternion.x;
    stateEstimate.qy = g_estimatedState.attitudeQuaternion.y;
    stateEstimate.qz = g_estimatedState.attitudeQuaternion.z;
    stateEstimate.qw = g_estimatedState.attitudeQuaternion.w;

    if (s_hasGyro) {
        gyro.x = s_lastGyro.data[0];
        gyro.y = s_lastGyro.data[1];
        gyro.z = s_lastGyro.data[2];
    }

    if (s_hasAcc) {
        acc.x = s_lastAcc.data[0];
        acc.y = s_lastAcc.data[1];
        acc.z = s_lastAcc.data[2];
    }

    if (s_hasBaro) {
        baro.pressure = s_lastBaro.data[0];
        baro.temp = s_lastBaro.data[1];
        baro.asl = s_lastBaro.data[2];
    }
}

void estimatorComplementary(uint32_t stabilizerStep)
{
    EstimatorMeasurement measurement;
    while (estimatorDequeue(&measurement)) {
        switch (measurement.type) {
        case MeasurementTypeGyroscope:
            s_lastGyro = measurement;
            s_hasGyro = true;
            break;
        case MeasurementTypeAcceleration:
            s_lastAcc = measurement;
            s_hasAcc = true;
            break;
        case MeasurementTypeBarometer:
            s_lastBaro = measurement;
            s_hasBaro = true;
            break;
        case MeasurementTypeTOF:
            s_lastTof = measurement;
            s_hasTof = true;
            break;
        default:
            break;
        }
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        if (s_hasGyro && s_hasAcc) {
            float gx_rad = s_lastGyro.data[0] * ((float)M_PI / 180.0f);
            float gy_rad = s_lastGyro.data[1] * ((float)M_PI / 180.0f);
            float gz_rad = s_lastGyro.data[2] * ((float)M_PI / 180.0f);

            sensfusion6UpdateQ(gx_rad, gy_rad, gz_rad,
                               s_lastAcc.data[0], s_lastAcc.data[1],
                               s_lastAcc.data[2], 1.0f / 250.0f);
        }

        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
        float qw_ = 1.0f;
        float qx_ = 0.0f;
        float qy_ = 0.0f;
        float qz_ = 0.0f;

        sensfusion6GetEulerRPY(&roll, &pitch, &yaw);
        sensfusion6GetQuaternion(&qw_, &qx_, &qy_, &qz_);

        g_estimatedState.attitude.roll = roll;
        g_estimatedState.attitude.pitch = pitch;
        g_estimatedState.attitude.yaw = yaw;

        g_estimatedState.attitudeQuaternion.w = qw_;
        g_estimatedState.attitudeQuaternion.x = qx_;
        g_estimatedState.attitudeQuaternion.y = qy_;
        g_estimatedState.attitudeQuaternion.z = qz_;

        if (s_hasAcc) {
            float accZNoGravity =
                sensfusion6GetAccZWithoutGravity(s_lastAcc.data[0],
                                                 s_lastAcc.data[1],
                                                 s_lastAcc.data[2]);
            g_estimatedState.acc.x = s_lastAcc.data[0];
            g_estimatedState.acc.y = s_lastAcc.data[1];
            g_estimatedState.acc.z = accZNoGravity;
            g_estimatedState.velocity.z += accZNoGravity * 9.80665f * (1.0f / 250.0f);
        }

        updateEstimatorLogs();
    }

    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
        g_estimatedState.position.z += g_estimatedState.velocity.z * 0.01f;
    }
}

void stateEstimatorStepInternal(uint32_t stabilizerStep, State *state, SensorData *sensors)
{
    (void)sensors;

    estimatorComplementary(stabilizerStep);
    if (state != NULL) {
        *state = g_estimatedState;
    }
}

bool commanderSetSetpoint(const Setpoint *setpoint, int priority)
{
    if (setpoint == NULL) {
        return false;
    }

    if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
        g_highLevelSetpointActive = false;
    }

    if (priority == COMMANDER_PRIORITY_DISABLE) {
        g_activeSetpoint = *setpoint;
        g_activePriority = priority;
        g_commanderLastUpdateTick = g_systemTickMs;
        thrustLocked = true;
        return true;
    }

    if (priority >= g_activePriority) {
        g_activeSetpoint = *setpoint;
        g_activePriority = priority;
        g_commanderLastUpdateTick = g_systemTickMs;

        if (priority == COMMANDER_PRIORITY_HIGHLEVEL) {
            g_highLevelSetpointActive = true;
        }

        return true;
    }

    return false;
}

void commanderRelaxPriority(void)
{
    g_activePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void)
{
    if (g_commanderLastUpdateTick == 0u) {
        return 0u;
    }

    return g_systemTickMs - g_commanderLastUpdateTick;
}

int commanderGetActivePriority(void)
{
    return g_activePriority;
}

void commanderGetSetpoint(Setpoint *out)
{
    if (out == NULL) {
        return;
    }

    *out = g_activeSetpoint;
}

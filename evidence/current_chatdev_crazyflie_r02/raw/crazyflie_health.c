/* DOCSTRING: Propeller/battery health-test state machine and variance helper. */
#include "6_generated_code.h"

#include <math.h>
#include <string.h>

TestState healthTestState = testDone;
uint8_t motorPass = 0u;
uint8_t batteryPass = 0u;
float batterySag = 0.0f;

static bool s_propRequestPending = false;
static bool s_batRequestPending = false;

static int s_sampleCount = 0;
static int s_motorIndex = 0;
static float s_noiseBuffer[PROPTEST_NBR_OF_VARIANCE_VALUES];
static float s_idleVoltage = 4.2f;
static float s_minLoadedVoltage = 4.2f;
static uint32_t s_batTick = 0u;
static uint32_t s_restartWaitStart = 0u;

void healthRequestPropTest(void)
{
    s_propRequestPending = true;
}

void healthRequestBatteryTest(void)
{
    s_batRequestPending = true;
}

bool healthShallWeRunTest(void)
{
    if (s_propRequestPending) {
        s_propRequestPending = false;
        healthTestState = configureAcc;
        motorPass = 0u;
        batteryPass = 0u;
        s_sampleCount = 0;
        s_motorIndex = 0;
        memset(s_noiseBuffer, 0, sizeof(s_noiseBuffer));
        s_idleVoltage = 4.2f;
        s_minLoadedVoltage = 4.2f;
        s_batTick = 0u;
        healthLog.motorPass = motorPass;
        healthLog.batteryPass = batteryPass;
        healthLog.batterySag = batterySag;
        return true;
    }

    if (s_batRequestPending) {
        s_batRequestPending = false;
        healthTestState = testBattery;
        s_batTick = 0u;
        s_idleVoltage = 4.2f;
        s_minLoadedVoltage = 4.2f;
        batterySag = 0.0f;
        batteryPass = 0u;
        healthLog.batteryPass = batteryPass;
        healthLog.batterySag = batterySag;
        return true;
    }

    return healthTestState != testDone;
}

static void healthUpdateLog(void)
{
    healthLog.motorPass = motorPass;
    healthLog.batteryPass = batteryPass;
    healthLog.batterySag = batterySag;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIndex)
{
    if (highThreshold == 0.0f) {
        return true;
    }

    if (measuredValue >= lowThreshold && measuredValue <= highThreshold) {
        motorPass |= (uint8_t)(1u << motorIndex);
        healthUpdateLog();
        return true;
    }

    healthLog.motorTestCount++;
    return false;
}

float variance(const float *buffer, int length)
{
    if (buffer == NULL || length <= 0) {
        return 0.0f;
    }

    double sum = 0.0;
    double sumSq = 0.0;
    for (int i = 0; i < length; ++i) {
        sum += (double)buffer[i];
        sumSq += (double)buffer[i] * (double)buffer[i];
    }

    double n = (double)length;
    return (float)(sumSq - (sum * sum) / n);
}

void healthRunTests(const SensorData *sensorData)
{
    if (sensorData == NULL) {
        return;
    }

    switch (healthTestState) {
    case configureAcc:
        motorPass = 0u;
        batteryPass = 0u;
        s_idleVoltage = 4.2f;
        s_minLoadedVoltage = 4.2f;
        s_sampleCount = 0;
        s_motorIndex = 0;
        healthTestState = measureNoiseFloor;
        break;

    case measureNoiseFloor:
        if (s_sampleCount < PROPTEST_NBR_OF_VARIANCE_VALUES) {
            s_noiseBuffer[s_sampleCount++] = sensorData->acc.x;
        } else {
            healthTestState = measureProp;
            s_motorIndex = 0;
        }
        break;

    case measureProp:
        if (s_motorIndex < 4) {
            /* The deterministic host model measures one sample per motor. */
            (void)evaluatePropTest(0.0f, 1.0f, sensorData->acc.x,
                                   (uint8_t)s_motorIndex);
            s_motorIndex++;
            if (s_motorIndex >= 4) {
                healthTestState = evaluatePropResult;
            }
        }
        break;

    case evaluatePropResult:
        healthTestState = testDone;
        healthUpdateLog();
        break;

    case testBattery:
        s_batTick++;
        if (s_batTick == 1u) {
            s_minLoadedVoltage = s_idleVoltage;
        } else if (s_batTick >= 2u && s_batTick <= 49u) {
            if (sensorData->baroPressure > 0.0f) {
                float loaded = sensorData->baroPressure / 1000.0f + 2.0f;
                if (loaded < s_minLoadedVoltage) {
                    s_minLoadedVoltage = loaded;
                }
            }
        } else if (s_batTick >= 50u) {
            batterySag = s_idleVoltage - s_minLoadedVoltage;
            if (batterySag < 0.0f) {
                batterySag = 0.0f;
            }
            batteryPass = (batterySag <= 0.8f) ? 1u : 0u;
            healthTestState = testDone;
            healthUpdateLog();
        }
        break;

    case evaluateBatResult:
        healthTestState = testDone;
        healthUpdateLog();
        break;

    case restartBatTest:
        if ((g_systemTickMs - s_restartWaitStart) >= 2000u) {
            healthTestState = testBattery;
            s_batTick = 0u;
        }
        break;

    case testDone:
        healthUpdateLog();
        break;

    default:
        break;
    }
}

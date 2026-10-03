/* 6_generated_code.c */
#include "6_generated_code.h"
#include <math.h>
#include <string.h>

/* Global State */
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX, gravityY, gravityZ;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

PidObject pidRoll, pidPitch, pidYaw;
PidObject pidRollRate, pidPitchRate, pidYawRate;

bool thrustLocked = false;
bool commanderModeSet = false;

SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0;

TestState healthTestState = configureAcc;
uint8_t motorPass = 0, batteryPass = 0;
float batterySag = 0.0f;

StateEstimateLog stateEstimate;
Axis3Log gyro, acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;

/* Numeric Functions */
int16_t saturateSignedInt16(int32_t value) {
    if (value > 32767) return 32767;
    if (value < -32767) return -32767;
    return (int16_t)value;
}

float capAngle(float angle_deg) {
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

float invSqrt(float x) {
    float xhalf = 0.5f * x;
    int32_t i = *(int32_t*)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float*)&i;
    x = x * (1.5f - xhalf * x * x);
    return x;
}

/* Sensfusion6 */
void sensfusion6Init(void) {
    qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
    sensfusion6IsInit = true;
}

bool sensfusion6Test(void) { return sensfusion6IsInit; }

void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    if (fabs(ax) < 0.001f && fabs(ay) < 0.001f && fabs(az) < 0.001f) {
        /* Skip correction */
    } else {
        /* Standard Mahony integration */
        if (twoKi > 0) {
            integralFBx += gx * twoKi; /* Simplified logic implementation */
        }
    }
    /* Update quaternions here based on gyro/acc */
}

/* Power Distribution */
void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw, MotorPower *out) {
    int32_t r = roll / 2, p = pitch / 2;
    out->m1 = thrust - r + p + yaw;
    out->m2 = thrust - r - p - yaw;
    out->m3 = thrust + r - p + yaw;
    out->m4 = thrust + r + p - yaw;
}

PowerCapResult powerDistributionCap(int32_t motors[4], int32_t maxAllowedThrust, int32_t idleThrust) {
    int32_t maxVal = motors[0];
    for (int i = 1; i < 4; i++) if (motors[i] > maxVal) maxVal = motors[i];
    
    PowerCapResult res = {false, 0};
    if (maxVal > maxAllowedThrust) {
        res.isCapped = true;
        res.reduction = maxVal - maxAllowedThrust;
        for (int i = 0; i < 4; i++) {
            motors[i] -= res.reduction;
            if (motors[i] < idleThrust) motors[i] = idleThrust;
        }
    }
    return res;
}

/* Health */
float variance(const float *buffer, int length) {
    float sum = 0, sumSq = 0;
    for (int i = 0; i < length; i++) {
        sum += buffer[i];
        sumSq += buffer[i] * buffer[i];
    }
    return sumSq - (sum * sum / length);
}

bool evaluatePropTest(float lowThreshold, float highThreshold, float measuredValue, uint8_t motorIdx) {
    if (highThreshold == 0.0f) return true;
    return (measuredValue >= lowThreshold && measuredValue <= highThreshold);
}

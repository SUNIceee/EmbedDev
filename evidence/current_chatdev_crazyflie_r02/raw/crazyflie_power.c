/* DOCSTRING: Power distribution, battery compensation, and explicit PWM conversion boundaries. */
#include "6_generated_code.h"

#include <math.h>

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out)
{
    if (out == NULL) {
        return;
    }

    int32_t r = (int32_t)roll / 2;
    int32_t p = (int32_t)pitch / 2;
    int32_t y = (int32_t)yaw;
    int32_t t = (int32_t)thrust;

    out->m1 = t - r + p + y;
    out->m2 = t - r - p - y;
    out->m3 = t + r - p + y;
    out->m4 = t + r + p - y;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4])
{
    if (motorForces == NULL) {
        return;
    }

    float thrustPart = 0.25f * thrustSi;
    float arm = 0.707106781f * armLength;
    float rollPart = 0.0f;
    float pitchPart = 0.0f;
    float yawPart = 0.0f;

    if (arm != 0.0f) {
        rollPart = (0.25f / arm) * torqueX;
        pitchPart = (0.25f / arm) * torqueY;
    }

    if (thrustToTorque != 0.0f) {
        yawPart = (0.25f / thrustToTorque) * torqueZ;
    }

    float f1 = thrustPart - rollPart + pitchPart + yawPart;
    float f2 = thrustPart - rollPart - pitchPart - yawPart;
    float f3 = thrustPart + rollPart - pitchPart + yawPart;
    float f4 = thrustPart + rollPart + pitchPart - yawPart;

    motorForces[0] = (f1 < 0.0f) ? 0.0f : f1;
    motorForces[1] = (f2 < 0.0f) ? 0.0f : f2;
    motorForces[2] = (f3 < 0.0f) ? 0.0f : f3;
    motorForces[3] = (f4 < 0.0f) ? 0.0f : f4;
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4])
{
    if (normalizedForces == NULL || motorPWMs == NULL) {
        return;
    }

    for (int i = 0; i < 4; ++i) {
        float f = normalizedForces[i];
        if (f < 0.0f) {
            f = 0.0f;
        }
        if (f > 1.0f) {
            f = 1.0f;
        }
        motorPWMs[i] = (uint16_t)(f * 65535.0f);
    }
}

static uint16_t motorForceToPwm(float force)
{
    if (force <= 0.0f) {
        return 0;
    }

    float ratio = force / CRAZYFLIE_MAX_MOTOR_FORCE_N;
    if (ratio >= 1.0f) {
        return 65535;
    }

    return (uint16_t)(ratio * 65535.0f);
}

void powerDistribution(const ControlData *control, MotorPower *motorPower)
{
    if (control == NULL || motorPower == NULL) {
        return;
    }

    if (control->controlMode == controlModeLegacy) {
        powerDistributionLegacy(control->thrust, control->roll, control->pitch,
                                control->yaw, motorPower);
        return;
    }

    if (control->controlMode == controlModeForceTorque) {
        float forces[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        powerDistributionForceTorque(control->thrustSi,
                                     control->torque.x,
                                     control->torque.y,
                                     control->torque.z,
                                     CRAZYFLIE_ARM_LENGTH_M,
                                     CRAZYFLIE_THRUST_TO_TORQUE,
                                     forces);
        motorPower->m1 = motorForceToPwm(forces[0]);
        motorPower->m2 = motorForceToPwm(forces[1]);
        motorPower->m3 = motorForceToPwm(forces[2]);
        motorPower->m4 = motorForceToPwm(forces[3]);
        return;
    }

    if (control->controlMode == controlModeForce) {
        uint16_t pwms[4] = {0u, 0u, 0u, 0u};
        powerDistributionForce(control->normalizedForces, pwms);
        motorPower->m1 = pwms[0];
        motorPower->m2 = pwms[1];
        motorPower->m3 = pwms[2];
        motorPower->m4 = pwms[3];
    }

    /* Unknown control mode intentionally leaves the output unchanged. */
}

int32_t capMinThrust(int32_t value, int32_t idleThrust)
{
    if (value < idleThrust) {
        return idleThrust;
    }
    return value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust)
{
    PowerCapResult result = {false, 0};

    if (motors == NULL) {
        return result;
    }

    int32_t maxValue = motors[0];
    for (int i = 1; i < 4; ++i) {
        if (motors[i] > maxValue) {
            maxValue = motors[i];
        }
    }

    if (maxValue <= maxAllowedThrust) {
        return result;
    }

    result.isCapped = true;
    result.reduction = maxValue - maxAllowedThrust;

    for (int i = 0; i < 4; ++i) {
        motors[i] = capMinThrust(motors[i] - result.reduction, idleThrust);
    }

    return result;
}

float batteryCompensation(float supplyVoltage, float filteredOld, float alpha)
{
    return filteredOld + alpha * (supplyVoltage - filteredOld);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust,
                                        float nominalVoltage,
                                        float actualVoltage)
{
    if (actualVoltage <= 0.0f || nominalVoltage <= 0.0f) {
        return motorThrust;
    }

    float compensated = ((float)motorThrust * nominalVoltage) / actualVoltage;
    if (compensated <= 0.0f) {
        return 0;
    }
    if (compensated >= 65535.0f) {
        return 65535;
    }

    return (uint16_t)(compensated + 0.5f);
}

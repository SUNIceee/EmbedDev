#include "6_generated_code.h"
#include <math.h>
#include <string.h>
#include <limits.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* -------------------------------------------------------------------------
 * Numeric helpers
 * ---------------------------------------------------------------------- */
int16_t saturateSignedInt16(int32_t value)
{
    if (value > 32767) return 32767;
    if (value < -32767) return -32767;
    return (int16_t)value;
}

float capAngle(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

/* -------------------------------------------------------------------------
 * Sensfusion6
 * ---------------------------------------------------------------------- */
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

void sensfusion6Init(void)
{
    if (sensfusion6IsInit) return;
    qw = 1.0f;
    qx = qy = qz = 0.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
    gravityX = 0.0f;
    gravityY = 0.0f;
    gravityZ = 1.0f;
    baseZacc = 0.0f;
    sensfusion6IsInit = true;
    sensfusion6IsCalibrated = false;
}

bool sensfusion6Test(void)
{
    return sensfusion6IsInit;
}

void estimatedGravityDirection(float qw_, float qx_, float qy_, float qz_,
                               float *gravX, float *gravY, float *gravZ)
{
    if (gravX) *gravX = 2.0f * (qx_ * qz_ - qw_ * qy_);
    if (gravY) *gravY = 2.0f * (qw_ * qx_ + qy_ * qz_);
    if (gravZ) *gravZ = qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_;
}

float invSqrt(float x)
{
    if (x <= 0.0f) return 0.0f;
    union {
        float f;
        int32_t i;
    } u;
    u.f = x;
    float xhalf = 0.5f * x;
    u.i = 0x5f3759df - (u.i >> 1);
    float y = u.f;
    y = y * (1.5f - xhalf * y * y);
    return y;
}

static void sensfusion6GyroIntegration(float gxr, float gyr, float gzr, float dt)
{
    float half_dt = 0.5f * dt;
    float qw_new = qw + (-qx * gxr - qy * gyr - qz * gzr) * half_dt;
    float qx_new = qx + ( qw * gxr + qy * gzr - qz * gyr) * half_dt;
    float qy_new = qy + ( qw * gyr - qx * gzr + qz * gxr) * half_dt;
    float qz_new = qz + ( qw * gzr + qx * gyr - qy * gxr) * half_dt;

    float qnorm = sqrtf(qw_new * qw_new + qx_new * qx_new +
                        qy_new * qy_new + qz_new * qz_new);
    if (qnorm < 1e-10f) return;

    qw = qw_new / qnorm;
    qx = qx_new / qnorm;
    qy = qy_new / qnorm;
    qz = qz_new / qnorm;
}

#if defined(CONFIG_IMU_MADGWICK_QUATERNION)
static void sensfusion6MadgwickUpdate(float gxr, float gyr, float gzr,
                                      float axn, float ayn, float azn,
                                      float dt)
{
    float q0 = qw, q1 = qx, q2 = qy, q3 = qz;

    float f1 = 2.0f * (q1 * q3 - q0 * q2) - axn;
    float f2 = 2.0f * (q0 * q1 + q2 * q3) - ayn;
    float f3 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - azn;

    float J11 = -2.0f * q2, J12 = 2.0f * q3, J13 = -2.0f * q0, J14 = 2.0f * q1;
    float J21 = 2.0f * q1, J22 = 2.0f * q0, J23 = 2.0f * q3, J24 = 2.0f * q2;
    float J31 = 0.0f, J32 = -4.0f * q1, J33 = -4.0f * q2, J34 = 0.0f;

    float grad0 = J11 * f1 + J21 * f2 + J31 * f3;
    float grad1 = J12 * f1 + J22 * f2 + J32 * f3;
    float grad2 = J13 * f1 + J23 * f2 + J33 * f3;
    float grad3 = J14 * f1 + J24 * f2 + J34 * f3;

    float gnorm = sqrtf(grad0 * grad0 + grad1 * grad1 +
                        grad2 * grad2 + grad3 * grad3);
    if (gnorm > 1e-10f) {
        grad0 /= gnorm;
        grad1 /= gnorm;
        grad2 /= gnorm;
        grad3 /= gnorm;
    } else {
        grad0 = grad1 = grad2 = grad3 = 0.0f;
    }

    float qDot0 = 0.5f * (-q1 * gxr - q2 * gyr - q3 * gzr) - beta * grad0;
    float qDot1 = 0.5f * ( q0 * gxr + q2 * gzr - q3 * gyr) - beta * grad1;
    float qDot2 = 0.5f * ( q0 * gyr - q1 * gzr + q3 * gxr) - beta * grad2;
    float qDot3 = 0.5f * ( q0 * gzr + q1 * gyr - q2 * gxr) - beta * grad3;

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    float rnorm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (rnorm < 1e-10f) return;

    qw = q0 / rnorm;
    qx = q1 / rnorm;
    qy = q2 / rnorm;
    qz = q3 / rnorm;
}
#endif

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt)
{
    float gxr = gx * (M_PI / 180.0f);
    float gyr = gy * (M_PI / 180.0f);
    float gzr = gz * (M_PI / 180.0f);

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    bool accelValid = norm > 1e-6f;
    float axn = 0.0f, ayn = 0.0f, azn = 0.0f;

    if (accelValid) {
        axn = ax / norm;
        ayn = ay / norm;
        azn = az / norm;
    }

#if defined(CONFIG_IMU_MADGWICK_QUATERNION)
    integralFBx = integralFBy = integralFBz = 0.0f;
    if (accelValid) {
        sensfusion6MadgwickUpdate(gxr, gyr, gzr, axn, ayn, azn, dt);
    } else {
        sensfusion6GyroIntegration(gxr, gyr, gzr, dt);
    }
#else
    if (twoKi <= 0.0f) {
        integralFBx = integralFBy = integralFBz = 0.0f;
    }

    float correctedGxr = gxr;
    float correctedGyr = gyr;
    float correctedGzr = gzr;

    if (accelValid) {
        float gx_est, gy_est, gz_est;
        estimatedGravityDirection(qw, qx, qy, qz, &gx_est, &gy_est, &gz_est);

        float ex = (ayn * gz_est - azn * gy_est);
        float ey = (azn * gx_est - axn * gz_est);
        float ez = (axn * gy_est - ayn * gx_est);

        if (twoKi > 0.0f) {
            integralFBx += twoKi * ex * dt;
            integralFBy += twoKi * ey * dt;
            integralFBz += twoKi * ez * dt;
        }

        correctedGxr += twoKp * ex + integralFBx;
        correctedGyr += twoKp * ey + integralFBy;
        correctedGzr += twoKp * ez + integralFBz;
    }

    sensfusion6GyroIntegration(correctedGxr, correctedGyr, correctedGzr, dt);
#endif

    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);

    if (!sensfusion6IsCalibrated && accelValid) {
        baseZacc = ax * gravityX + ay * gravityY + az * gravityZ;
        sensfusion6IsCalibrated = true;
    }
}

void sensfusion6GetQuaternion(float *qw_out, float *qx_out, float *qy_out, float *qz_out)
{
    if (qw_out) *qw_out = qw;
    if (qx_out) *qx_out = qx;
    if (qy_out) *qy_out = qy;
    if (qz_out) *qz_out = qz;
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float gx, gy, gz;
    estimatedGravityDirection(qw, qx, qy, qz, &gx, &gy, &gz);

    float clipped = gx;
    if (clipped > 1.0f) clipped = 1.0f;
    if (clipped < -1.0f) clipped = -1.0f;

    if (roll_deg) *roll_deg = atan2f(gy, gz) * (180.0f / M_PI);
    if (pitch_deg) *pitch_deg = asinf(-clipped) * (180.0f / M_PI);
    if (yaw_deg) *yaw_deg = atan2f(2.0f * (qw * qz + qx * qy),
                                    1.0f - 2.0f * (qy * qy + qz * qz)) * (180.0f / M_PI);
}

float sensfusion6GetAccZ(float ax, float ay, float az)
{
    float gx, gy, gz;
    estimatedGravityDirection(qw, qx, qy, qz, &gx, &gy, &gz);
    return ax * gx + ay * gy + az * gz;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az)
{
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

/* -------------------------------------------------------------------------
 * Power distribution and battery
 * ---------------------------------------------------------------------- */
void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out)
{
    if (!out) return;
    int32_t r = roll / 2;
    int32_t p = pitch / 2;
    out->m1 = (int32_t)thrust - r + p + yaw;
    out->m2 = (int32_t)thrust - r - p - yaw;
    out->m3 = (int32_t)thrust + r - p + yaw;
    out->m4 = (int32_t)thrust + r + p - yaw;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4])
{
    if (!motorForces) return;

    float thrustPart = 0.25f * thrustSi;
    float rollPart = 0.0f;
    float pitchPart = 0.0f;
    float yawPart = 0.0f;

    if (armLength > 1e-8f) {
        float arm = 0.707106781f * armLength;
        rollPart = 0.25f / arm * torqueX;
        pitchPart = 0.25f / arm * torqueY;
    }
    if (thrustToTorque > 1e-8f) {
        yawPart = 0.25f / thrustToTorque * torqueZ;
    }

    motorForces[0] = thrustPart - rollPart - pitchPart - yawPart;
    motorForces[1] = thrustPart - rollPart + pitchPart + yawPart;
    motorForces[2] = thrustPart + rollPart - pitchPart + yawPart;
    motorForces[3] = thrustPart + rollPart + pitchPart - yawPart;

    for (int i = 0; i < 4; i++) {
        if (motorForces[i] < 0.0f) motorForces[i] = 0.0f;
    }
}

static uint16_t motorForceToPwm(float force)
{
    if (force <= 0.0f) return 0U;
    if (force >= CRAZYFLIE_MAX_MOTOR_FORCE_N) return 65535U;
    return (uint16_t)((force / CRAZYFLIE_MAX_MOTOR_FORCE_N) * 65535.0f + 0.5f);
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4])
{
    if (!normalizedForces || !motorPWMs) return;
    for (int i = 0; i < 4; i++) {
        float f = normalizedForces[i];
        if (f < 0.0f) f = 0.0f;
        if (f > 1.0f) f = 1.0f;
        motorPWMs[i] = (uint16_t)(f * 65535.0f);
    }
}

void powerDistribution(const ControlData *control, MotorPower *motorPower)
{
    if (!control || !motorPower) return;

    switch (control->controlMode) {
        case controlModeLegacy:
            powerDistributionLegacy(control->thrust, control->roll,
                                    control->pitch, control->yaw, motorPower);
            break;
        case controlModeForceTorque: {
            float forces[4] = {0};
            powerDistributionForceTorque(control->thrustSi,
                                         control->torque.x, control->torque.y,
                                         control->torque.z,
                                         CRAZYFLIE_ARM_LENGTH_M,
                                         CRAZYFLIE_THRUST_TO_TORQUE,
                                         forces);
            motorPower->m1 = motorForceToPwm(forces[0]);
            motorPower->m2 = motorForceToPwm(forces[1]);
            motorPower->m3 = motorForceToPwm(forces[2]);
            motorPower->m4 = motorForceToPwm(forces[3]);
            break;
        }
        case controlModeForce: {
            uint16_t pwm[4] = {0};
            powerDistributionForce(control->normalizedForces, pwm);
            motorPower->m1 = pwm[0];
            motorPower->m2 = pwm[1];
            motorPower->m3 = pwm[2];
            motorPower->m4 = pwm[3];
            break;
        }
        default:
            break;
    }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust)
{
    return value < idleThrust ? idleThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust)
{
    PowerCapResult result = { false, 0 };
    if (!motors) return result;

    int32_t maxVal = motors[0];
    for (int i = 1; i < 4; i++) {
        if (motors[i] > maxVal) maxVal = motors[i];
    }

    if (maxVal <= maxAllowedThrust) return result;

    int32_t reduction = maxVal - maxAllowedThrust;
    for (int i = 0; i < 4; i++) {
        motors[i] -= reduction;
        motors[i] = capMinThrust(motors[i], idleThrust);
    }

    result.isCapped = true;
    result.reduction = reduction;
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
    if (actualVoltage <= 0.0f) return motorThrust;
    float result = (float)motorThrust * (nominalVoltage / actualVoltage);
    if (result < 0.0f) result = 0.0f;
    if (result > 65535.0f) result = 65535.0f;
    return (uint16_t)(result + 0.5f);
}

/* -------------------------------------------------------------------------
 * Cascade PID and controller
 * ---------------------------------------------------------------------- */
PidObject pidRoll = {0};
PidObject pidPitch = {0};
PidObject pidYaw = {0};
PidObject pidRollRate = {0};
PidObject pidPitchRate = {0};
PidObject pidYawRate = {0};

static float pidUpdateDt = 0.002f;

static float pidUpdate(PidObject *pid, float actual, float desired, float dt)
{
    if (!pid) return 0.0f;
    if (dt <= 0.0f) dt = 0.002f;

    float error = desired - actual;
    float p = pid->kp * error;
    pid->integral += pid->ki * error * dt;
    float d = pid->kd * (error - pid->prevError) / dt;
    float output = p + pid->integral + d + pid->kff * desired;
    pid->prevError = error;
    pid->output = output;
    return output;
}

void attitudeControllerInit(float updateDt)
{
    static bool initialized = false;
    if (initialized) return;

    memset(&pidRoll, 0, sizeof(pidRoll));
    memset(&pidPitch, 0, sizeof(pidPitch));
    memset(&pidYaw, 0, sizeof(pidYaw));
    memset(&pidRollRate, 0, sizeof(pidRollRate));
    memset(&pidPitchRate, 0, sizeof(pidPitchRate));
    memset(&pidYawRate, 0, sizeof(pidYawRate));

    pidRoll.initialized = true;
    pidPitch.initialized = true;
    pidYaw.initialized = true;
    pidRollRate.initialized = true;
    pidPitchRate.initialized = true;
    pidYawRate.initialized = true;

    pidUpdateDt = updateDt > 0.0f ? updateDt : 0.002f;
    initialized = true;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired)
{
    float out;
    out = pidUpdate(&pidRollRate, rollActual, rollDesired, pidUpdateDt);
    pidRollRate.output = saturateSignedInt16((int32_t)out);

    out = pidUpdate(&pidPitchRate, pitchActual, pitchDesired, pidUpdateDt);
    pidPitchRate.output = saturateSignedInt16((int32_t)out);

    out = pidUpdate(&pidYawRate, yawActual, yawDesired, pidUpdateDt);
    pidYawRate.output = saturateSignedInt16((int32_t)out);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired)
{
    pidUpdate(&pidRoll, rollActual, rollDesired, pidUpdateDt);
    pidUpdate(&pidPitch, pitchActual, pitchDesired, pidUpdateDt);

    float yawError = capAngle(yawDesired - yawActual);
    pidUpdate(&pidYaw, yawActual, yawActual + yawError, pidUpdateDt);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual)
{
    (void)rollActual; (void)pitchActual; (void)yawActual;
    pidRoll.integral = 0.0f;  pidRoll.prevError = 0.0f;  pidRoll.output = 0.0f;
    pidPitch.integral = 0.0f; pidPitch.prevError = 0.0f; pidPitch.output = 0.0f;
    pidYaw.integral = 0.0f;   pidYaw.prevError = 0.0f;   pidYaw.output = 0.0f;
    pidRollRate.integral = 0.0f;  pidRollRate.prevError = 0.0f;  pidRollRate.output = 0.0f;
    pidPitchRate.integral = 0.0f; pidPitchRate.prevError = 0.0f; pidPitchRate.output = 0.0f;
    pidYawRate.integral = 0.0f;   pidYawRate.prevError = 0.0f;   pidYawRate.output = 0.0f;
}

void attitudeControllerResetRollAttitudePID(float rollActual)
{
    (void)rollActual;
    pidRoll.integral = 0.0f;
    pidRoll.prevError = 0.0f;
    pidRoll.output = 0.0f;
}

void attitudeControllerResetPitchAttitudePID(float pitchActual)
{
    (void)pitchActual;
    pidPitch.integral = 0.0f;
    pidPitch.prevError = 0.0f;
    pidPitch.output = 0.0f;
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw)
{
    if (roll) *roll = (int16_t)pidRollRate.output;
    if (pitch) *pitch = (int16_t)pidPitchRate.output;
    if (yaw) *yaw = (int16_t)pidYawRate.output;
}

static PidObject pidZPosition = {0};

static void positionControllerReset(void)
{
    pidZPosition.integral = 0.0f;
    pidZPosition.prevError = 0.0f;
    pidZPosition.output = 0.0f;
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state)
{
    if (!setpoint || !state) return 0U;

    if (!pidZPosition.initialized) {
        pidZPosition.kp = 1.0f;
        pidZPosition.ki = 0.0f;
        pidZPosition.kd = 0.0f;
        pidZPosition.kff = 0.0f;
        pidZPosition.integral = 0.0f;
        pidZPosition.prevError = 0.0f;
        pidZPosition.output = 0.0f;
        pidZPosition.initialized = true;
    }

    float error = 0.0f;
    if (setpoint->mode.z == modeAbs) {
        error = setpoint->position.z - state->position.z;
    } else if (setpoint->mode.z == modeVelocity) {
        error = setpoint->velocity.z - state->velocity.z;
    }

    float out = pidZPosition.kp * error + pidZPosition.kd * (-state->velocity.z);
    if (out < 0.0f) out = 0.0f;
    if (out > (float)MAX_THRUST) out = (float)MAX_THRUST;
    return (uint16_t)(out + 0.5f);
}

static float controllerDesiredYaw = 0.0f;
static bool controllerDesiredYawInitialized = false;

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt)
{
    if (!sensors || !setpoint || !state || !control) return;

    if (!controllerDesiredYawInitialized) {
        controllerDesiredYaw = state->attitude.yaw;
        controllerDesiredYawInitialized = true;
    }

    control->thrustSi = 0.0f;
    memset(control->normalizedForces, 0, sizeof(control->normalizedForces));

    if (setpoint->thrust == 0U) {
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->thrust = 0U;
        control->controlMode = controlModeLegacy;
        attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch,
                                      state->attitude.yaw);
        positionControllerReset();
        controllerDesiredYaw = state->attitude.yaw;
        return;
    }

    if (setpoint->mode.quat == modeAbs) {
        Quaternion q = setpoint->attitudeQuaternion;
        controllerDesiredYaw = atan2f(2.0f * (q.w * q.z + q.x * q.y),
                                       1.0f - 2.0f * (q.y * q.y + q.z * q.z)) *
                               (180.0f / M_PI);
    } else if (setpoint->mode.yaw == modeVelocity) {
        controllerDesiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
        if (yawMaxDelta != 0.0f) {
            float delta = capAngle(controllerDesiredYaw - state->attitude.yaw);
            if (delta > yawMaxDelta) controllerDesiredYaw = state->attitude.yaw + yawMaxDelta;
            if (delta < -yawMaxDelta) controllerDesiredYaw = state->attitude.yaw - yawMaxDelta;
        }
    } else if (setpoint->mode.yaw == modeAbs) {
        controllerDesiredYaw = setpoint->attitude.yaw;
    }

    float rollDesired = setpoint->attitude.roll;
    float pitchDesired = setpoint->attitude.pitch;

    if (setpoint->mode.roll == modeVelocity) {
        rollDesired = setpoint->attitudeRate.roll;
        attitudeControllerResetRollAttitudePID(state->attitude.roll);
    }
    if (setpoint->mode.pitch == modeVelocity) {
        pitchDesired = setpoint->attitudeRate.pitch;
        attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
    }

    attitudeControllerCorrectAttitudePID(state->attitude.roll, rollDesired,
                                         state->attitude.pitch, pitchDesired,
                                         state->attitude.yaw, controllerDesiredYaw);

    float rollActual = sensors->gyro.x;
    float pitchActual = -sensors->gyro.y;
    float yawActual = sensors->gyro.z;

    attitudeControllerCorrectRatePID(rollActual, pidRoll.output,
                                     pitchActual, pidPitch.output,
                                     yawActual, pidYaw.output);

    control->roll = (int16_t)pidRollRate.output;
    control->pitch = (int16_t)pidPitchRate.output;
    control->yaw = -(int16_t)pidYawRate.output;

    if (setpoint->mode.z == modeDisable) {
        control->thrust = setpoint->thrust;
    } else {
        control->thrust = positionControllerUpdate(setpoint, state);
    }
    if (control->thrust > MAX_THRUST) control->thrust = MAX_THRUST;

    control->controlMode = controlModeLegacy;
}

/* -------------------------------------------------------------------------
 * CRTP Commander RPYT
 * ---------------------------------------------------------------------- */
bool thrustLocked = false;
bool commanderModeSet = false;

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime)
{
    if (!rollPrime || !pitchPrime) return;
    float rad = yaw_deg * (M_PI / 180.0f);
    float c = cosf(rad);
    float s = sinf(rad);
    *rollPrime = roll * c - pitch * s;
    *pitchPrime = roll * s + pitch * c;
}

static bool altHoldActive = false;

void crtpCommanderRpytDecodeSetpoint(
    const CommanderCrtpLegacyValues *values,
    Setpoint *setpoint,
    bool altHoldMode,
    bool posHoldMode,
    bool posSetMode,
    StabilizationType stabilizationModeRoll,
    StabilizationType stabilizationModePitch,
    StabilizationType stabilizationModeYaw,
    YawMode yawMode)
{
    if (!values || !setpoint) return;

    memset(setpoint, 0, sizeof(*setpoint));

    if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) {
        thrustLocked = true;
    }
    if (values->thrust == 0U) {
        thrustLocked = false;
    }

    if (altHoldMode) {
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0U;
        setpoint->velocity.z = ((float)values->thrust - 32767.0f) / 32767.0f;
        if (!altHoldActive) {
            positionControllerReset();
            altHoldActive = true;
            commanderModeSet = true;
        }
    } else {
        if (altHoldActive) {
            altHoldActive = false;
            commanderModeSet = false;
            setpoint->mode.z = modeDisable;
        }
        if (thrustLocked || values->thrust < MIN_THRUST) {
            setpoint->thrust = 0U;
        } else {
            setpoint->thrust = values->thrust > MAX_THRUST ? MAX_THRUST : values->thrust;
        }
    }

    bool posSetHandled = false;
    bool posHoldHandled = false;

    if (posSetMode && values->thrust != 0U) {
        setpoint->mode.x = modeAbs;
        setpoint->mode.y = modeAbs;
        setpoint->mode.z = modeAbs;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->mode.yaw = modeAbs;
        setpoint->position.x = -values->pitch;
        setpoint->position.y = values->roll;
        setpoint->position.z = ((float)values->thrust) / 1000.0f;
        setpoint->attitude.yaw = values->yaw;
        setpoint->thrust = 0U;
        posSetHandled = true;
    }

    if (!posSetHandled && posHoldMode) {
        setpoint->mode.x = modeVelocity;
        setpoint->mode.y = modeVelocity;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->velocity.x = values->pitch / 30.0f;
        setpoint->velocity.y = values->roll / 30.0f;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        posHoldHandled = true;
    }

    if (yawMode == CAREFREE) {
        memset(setpoint, 0, sizeof(*setpoint));
        return;
    }

    float rollCmd = values->roll;
    float pitchCmd = values->pitch;
    if (yawMode == PLUSMODE) {
        rotateYaw(values->roll, values->pitch, 45.0f, &rollCmd, &pitchCmd);
    }

    if (!posSetHandled && !posHoldHandled) {
        if (stabilizationModeRoll == RATE) {
            setpoint->mode.roll = modeVelocity;
            setpoint->attitudeRate.roll = rollCmd;
        } else {
            setpoint->mode.roll = modeAbs;
            setpoint->attitude.roll = rollCmd;
        }

        if (stabilizationModePitch == RATE) {
            setpoint->mode.pitch = modeVelocity;
            setpoint->attitudeRate.pitch = pitchCmd;
        } else {
            setpoint->mode.pitch = modeAbs;
            setpoint->attitude.pitch = pitchCmd;
        }
    }

    if (!posSetHandled) {
        if (stabilizationModeYaw == RATE) {
            setpoint->mode.yaw = modeVelocity;
            setpoint->attitudeRate.yaw = -values->yaw;
        } else {
            setpoint->mode.yaw = modeAbs;
            setpoint->attitude.yaw = values->yaw;
        }
    }
}

/* -------------------------------------------------------------------------
 * Supervisor
 * ---------------------------------------------------------------------- */
SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0U;

static uint32_t supervisorCurrentTickMs = 0U;
static bool supervisorArmed = false;
static bool supervisorCrashed = false;
static bool supervisorSensorsActive = false;

static SensorData supervisorSensors = {0};
static uint32_t supervisorMotorRatioInput[4] = {0};
static uint32_t supervisorIdleThrust = 0U;
static int32_t supervisorMotorRpmInput[4] = {0};

static float cfgCrashDetectionGs = 0.0f;
static float cfgFreeFallThreshold = 0.0f;
static float cfgAcceptedTiltAccZ = 0.0f;
static float cfgAcceptedUpsideDownAccZ = 0.0f;
static uint32_t cfgMaxTiltTime = 0U;
static uint32_t cfgMaxUpsideDownTime = 0U;
static bool cfgTumbleCheckEnabled = true;

static int32_t cfgRpmCheckMin = 1;
static int32_t cfgRpmCheckMax = INT32_MAX;
static int32_t cfgRpmNotRespondingThreshold = 0;
static uint32_t cfgRpmNotRespondingDuration = 1000U;

static uint32_t spinupStartTick = 0U;
static bool spinupStartPending = false;
static bool lastTumbledResult = false;
static bool lastCrashDetected = false;
static uint32_t latestArmingTick = 0U;
static uint32_t latestLandingTick = 0U;
static uint32_t cfgPreflightTimeoutDuration = 0U;
static uint32_t cfgLandingTimeoutDuration = 0U;

static bool trajectoryFlying = false;
static bool trajectoryFinished = false;
static bool trajectoryDisabled = false;
static bool deckFault = false;

static bool flyingSeen = false;
static uint32_t lastFlightTick = 0U;
static uint32_t tumbleStartTick = 0U;
static uint32_t notRespondingStart = 0U;

static bool s_crtpEmergencyStop = false;
static bool s_paramEmergencyStop = false;
static bool s_emergencyStopWatchdogFailed = false;

void supervisorInit(void)
{
    supervisorState = supervisorStateLocked;
    supervisorConditionBits = 0U;
    supervisorArmed = false;
    supervisorCrashed = false;
    supervisorSensorsActive = false;
    memset(&supervisorSensors, 0, sizeof(supervisorSensors));
    memset(supervisorMotorRatioInput, 0, sizeof(supervisorMotorRatioInput));
    memset(supervisorMotorRpmInput, 0, sizeof(supervisorMotorRpmInput));
    supervisorIdleThrust = 0U;

    cfgCrashDetectionGs = 0.0f;
    cfgFreeFallThreshold = 0.0f;
    cfgAcceptedTiltAccZ = 0.0f;
    cfgAcceptedUpsideDownAccZ = 0.0f;
    cfgMaxTiltTime = 0U;
    cfgMaxUpsideDownTime = 0U;
    cfgTumbleCheckEnabled = true;
    cfgRpmCheckMin = 1;
    cfgRpmCheckMax = INT32_MAX;
    cfgRpmNotRespondingThreshold = 0;
    cfgRpmNotRespondingDuration = 1000U;

    spinupStartTick = 0U;
    spinupStartPending = false;
    lastTumbledResult = false;
    lastCrashDetected = false;
    latestArmingTick = 0U;
    latestLandingTick = 0U;
    cfgPreflightTimeoutDuration = 0U;
    cfgLandingTimeoutDuration = 0U;

    trajectoryFlying = false;
    trajectoryFinished = false;
    trajectoryDisabled = false;
    deckFault = false;

    flyingSeen = false;
    lastFlightTick = 0U;
    tumbleStartTick = 0U;
    notRespondingStart = 0U;

    s_crtpEmergencyStop = false;
    s_paramEmergencyStop = false;
    s_emergencyStopWatchdogFailed = false;
    supervisorCurrentTickMs = 0U;
}

static bool supervisorStateKeepsArming(SupervisorState s)
{
    return s == supervisorStateArming ||
           s == supervisorStateReadyToFly ||
           s == supervisorStateFlying ||
           s == supervisorStateWarningLevelOut ||
           s == supervisorStateLanded;
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
    return supervisorArmed;
}

bool supervisorIsCrashed(void)
{
    return supervisorCrashed;
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
    uint16_t bits = 0U;
    if (supervisorCanArm()) bits |= (1U << 0);
    if (supervisorArmed) bits |= (1U << 1);
    if (autoArmingEnabled) bits |= (1U << 2);
    if (supervisorCanFly()) bits |= (1U << 3);
    if (supervisorConditionBits & SUPERVISOR_CB_IS_FLYING) bits |= (1U << 4);
    if (supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) bits |= (1U << 5);
    if (supervisorState == supervisorStateLocked) bits |= (1U << 6);
    if (supervisorCrashed) bits |= (1U << 7);
    if (trajectoryFlying) bits |= (1U << 8);
    if (trajectoryFinished) bits |= (1U << 9);
    if (trajectoryDisabled) bits |= (1U << 10);
    if (deckFault) bits |= (1U << 11);
    return bits;
}

bool supervisorRequestArming(bool doArm)
{
    if (doArm) {
        if (supervisorArmed) return true;
        if (!supervisorCanArm()) return false;
        supervisorArmed = true;
        supervisorState = supervisorStateArming;
        supervisorConditionBits |= SUPERVISOR_CB_ARMED;
        spinupStartPending = true;
        return true;
    } else {
        if (!supervisorArmed) return true;
        supervisorArmed = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
        spinupStartPending = false;
        spinupStartTick = 0U;
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
        return true;
    }
}

bool supervisorRequestCrashRecovery(bool doRecovery)
{
    if (doRecovery) {
        if (lastTumbledResult) return false;
        supervisorCrashed = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
        if (supervisorState == supervisorStateCrashed) {
            supervisorState = supervisorStateReadyToFly;
        }
        return true;
    } else {
        supervisorCrashed = true;
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        supervisorState = supervisorStateCrashed;
        return true;
    }
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick)
{
    if (!motorRatios) return false;

    bool anyAbove = false;
    for (int i = 0; i < 4; i++) {
        if (motorRatios[i] > idleThrust) anyAbove = true;
    }
    if (anyAbove) {
        lastFlightTick = currentTick;
        flyingSeen = true;
    }
    if (!flyingSeen) return false;

    return (currentTick - lastFlightTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling)
{
    bool freeFall = false;
    bool tumbled = false;
    bool crashNow = false;

    if (crashDetectionGs > 0.0f) {
        float norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
        crashNow = fabsf(norm - 1.0f) > crashDetectionGs;
    }

    lastCrashDetected = crashNow;
    if (crashNow) {
        supervisorCrashed = true;
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
    }

    if (fabsf(accX) < freeFallThreshold &&
        fabsf(accY) < freeFallThreshold &&
        fabsf(accZ) < freeFallThreshold) {
        freeFall = true;
        tumbleStartTick = 0U;
    }

    if (isFreeFalling) *isFreeFalling = freeFall;

    if (freeFall) {
        lastTumbledResult = false;
        return false;
    }

    if (!tumbleCheckEnabled) {
        lastTumbledResult = false;
        return false;
    }

    if (accZ >= acceptedTiltAccZ) {
        tumbleStartTick = 0U;
    } else {
        if (tumbleStartTick == 0U) {
            tumbleStartTick = currentTick;
        }
        uint32_t timeout = acceptedUpsideDownAccZ < acceptedTiltAccZ &&
                           accZ < acceptedUpsideDownAccZ
                               ? maxUpsideDownTime
                               : maxTiltTime;
        if (timeout > 0U && (currentTick - tumbleStartTick) >= timeout) {
            tumbled = true;
        }
    }

    lastTumbledResult = tumbled;
    return tumbled;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick)
{
    if (lastNotificationTick == 0U) return true;
    return (currentTick - lastNotificationTick) < DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration)
{
    if (state != supervisorStateReadyToFly) return false;
    if (latestArmingTick == 0U) return false;
    return (currentTick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTick,
                                uint32_t landingTimeoutDuration)
{
    if (latestLandingTick == 0U) return false;
    return (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed)
{
    s_crtpEmergencyStop = crtpEmergencyStop;
    s_paramEmergencyStop = paramEmergencyStop;
    s_emergencyStopWatchdogFailed = emergencyStopWatchdogFailed;

    if (s_crtpEmergencyStop || s_paramEmergencyStop || s_emergencyStopWatchdogFailed) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
    }
    return supervisorConditionBits;
}

static bool supervisorHasFault(uint32_t bits)
{
    return (bits & (SUPERVISOR_CB_EMERGENCY_STOP |
                    SUPERVISOR_CB_IS_TUMBLED |
                    SUPERVISOR_CB_FREE_FALL |
                    SUPERVISOR_CB_MOTORS_NOT_RESPONDING |
                    SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT |
                    SUPERVISOR_CB_CRASHED)) != 0U;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t supervisorConditionBits_,
                                SupervisorState state)
{
    if (!setpoint) return;

    if (state == supervisorStateWarningLevelOut) {
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.roll = modeAbs;
        setpoint->mode.pitch = modeAbs;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = 0.0f;
        return;
    }

    if (state == supervisorStateArming ||
        state == supervisorStateReadyToFly ||
        state == supervisorStateFlying ||
        state == supervisorStateLanded) {
        if (supervisorHasFault(supervisorConditionBits_)) {
            memset(setpoint, 0, sizeof(*setpoint));
        }
        return;
    }

    memset(setpoint, 0, sizeof(*setpoint));
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax)
{
    if (!motorRPMs) return false;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) return false;
    }
    return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly,
                           uint32_t currentTick)
{
    if (!motorRPMs || !canFly) {
        notRespondingStart = 0U;
        return false;
    }

    bool allLow = true;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] >= rpmThreshold) {
            allLow = false;
            break;
        }
    }

    if (!allLow) {
        notRespondingStart = 0U;
        return false;
    }

    if (notRespondingStart == 0U) {
        notRespondingStart = currentTick;
    }
    return (currentTick - notRespondingStart) >= rpmCheckDurationMs;
}

void supervisorSetSensorData(const SensorData *sensors)
{
    if (sensors) {
        supervisorSensors = *sensors;
        supervisorSensorsActive = true;
    }
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust)
{
    if (motorRatios) {
        for (int i = 0; i < 4; i++) supervisorMotorRatioInput[i] = motorRatios[i];
    }
    supervisorIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4])
{
    if (motorRPMs) {
        for (int i = 0; i < 4; i++) supervisorMotorRpmInput[i] = motorRPMs[i];
    }
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled)
{
    cfgCrashDetectionGs = crashDetectionGs;
    cfgFreeFallThreshold = freeFallThreshold;
    cfgAcceptedTiltAccZ = acceptedTiltAccZ;
    cfgAcceptedUpsideDownAccZ = acceptedUpsideDownAccZ;
    cfgMaxTiltTime = maxTiltTime;
    cfgMaxUpsideDownTime = maxUpsideDownTime;
    cfgTumbleCheckEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs)
{
    autoArmingEnabled = autoArming;
    spinupTimeoutDuration = spinupTimeoutDurationMs;
}

void supervisorUpdate(uint32_t stabilizerStep)
{
    uint32_t currentTick = supervisorCurrentTickMs++;
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;

    (void)updateAndPopulateConditions(s_crtpEmergencyStop, s_paramEmergencyStop,
                                      s_emergencyStopWatchdogFailed);

    SupervisorState previous = supervisorState;
    bool sensorsOk = supervisorSensorsActive;

    if (previous == supervisorStateLocked ||
        previous == supervisorStatePreFlChecksNotPassed) {
        if (sensorsOk && !supervisorCrashed &&
            !(supervisorConditionBits & (SUPERVISOR_CB_CRASHED |
                                         SUPERVISOR_CB_IS_TUMBLED |
                                         SUPERVISOR_CB_FREE_FALL |
                                         SUPERVISOR_CB_EMERGENCY_STOP))) {
            supervisorState = supervisorStatePreFlChecksPassed;
        } else {
            supervisorState = supervisorStatePreFlChecksNotPassed;
        }
    }

    bool fly = isFlyingCheck(supervisorMotorRatioInput, supervisorIdleThrust, currentTick);
    bool freeFall = false;
    bool tumbled = isTumbledCheck(supervisorSensors.acc.x, supervisorSensors.acc.y,
                                  supervisorSensors.acc.z,
                                  cfgCrashDetectionGs, cfgFreeFallThreshold,
                                  cfgAcceptedTiltAccZ, cfgAcceptedUpsideDownAccZ,
                                  cfgMaxTiltTime, cfgMaxUpsideDownTime,
                                  cfgTumbleCheckEnabled, currentTick, &freeFall);

    if (fly) supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
    else supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;

    if (tumbled) supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
    else supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;

    if (freeFall) supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
    else supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;

    if (lastCrashDetected) {
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        supervisorCrashed = true;
    }

    bool rpmValid = isRPMatArmingValid(supervisorMotorRpmInput, cfgRpmCheckMin, cfgRpmCheckMax);
    if (rpmValid) supervisorConditionBits |= SUPERVISOR_CB_RPM_AT_ARMING_VALID;
    else supervisorConditionBits &= ~SUPERVISOR_CB_RPM_AT_ARMING_VALID;

    bool motorsNotResp = isMotorsNotResponding(supervisorMotorRpmInput,
                                               cfgRpmNotRespondingThreshold,
                                               cfgRpmNotRespondingDuration,
                                               supervisorCanFly(), currentTick);
    if (motorsNotResp) supervisorConditionBits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    else supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;

    uint32_t inactivity = commanderGetInactivityTime();
    if (inactivity > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    }
    if (inactivity > COMMANDER_WDT_TIMEOUT_STABILIZE) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    }

    if (autoArmingEnabled &&
        previous != supervisorStatePreFlChecksPassed &&
        supervisorState == supervisorStatePreFlChecksPassed) {
        supervisorRequestArming(true);
    }

    if (freeFall) {
        supervisorState = supervisorStateExceptFreeFall;
    } else if (tumbled || lastCrashDetected) {
        supervisorState = supervisorStateCrashed;
    } else {
        switch (supervisorState) {
        case supervisorStateArming:
            if (spinupStartPending) {
                spinupStartTick = currentTick;
                spinupStartPending = false;
                latestArmingTick = currentTick;
            } else if (spinupStartTick == 0U && previous != supervisorStateArming) {
                spinupStartTick = currentTick;
                latestArmingTick = currentTick;
            }

            if (spinupStartTick != 0U && spinupTimeoutDuration > 0U &&
                (currentTick - spinupStartTick) >= spinupTimeoutDuration) {
                supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
            }
            break;

        case supervisorStateReadyToFly:
        case supervisorStateWarningLevelOut:
        case supervisorStateFlying:
        case supervisorStateLanded:
            if (inactivity > COMMANDER_WDT_TIMEOUT_STABILIZE &&
                inactivity <= COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
                supervisorState = supervisorStateWarningLevelOut;
            } else if (supervisorState == supervisorStateWarningLevelOut) {
                supervisorState = supervisorStateReadyToFly;
            }

            if (fly) {
                supervisorState = supervisorStateFlying;
            } else if (supervisorState == supervisorStateFlying) {
                supervisorState = supervisorStateLanded;
                latestLandingTick = currentTick;
            }
            break;

        default:
            break;
        }
    }

    if (supervisorState != supervisorStateArming) {
        spinupStartTick = 0U;
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    }

    bool previousKeeper = supervisorStateKeepsArming(previous);
    bool currentKeeper = supervisorStateKeepsArming(supervisorState);
    if (previousKeeper && !currentKeeper) {
        supervisorArmed = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    }
    if (supervisorArmed) supervisorConditionBits |= SUPERVISOR_CB_ARMED;

    if (supervisorIsPreflightTimeout(supervisorState, latestArmingTick, currentTick,
                                     cfgPreflightTimeoutDuration)) {
        supervisorConditionBits |= SUPERVISOR_CB_PREFLIGHT_TIMEOUT;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_PREFLIGHT_TIMEOUT;
    }

    if (supervisorIsLandingTimeout(latestLandingTick, currentTick,
                                   cfgLandingTimeoutDuration)) {
        supervisorConditionBits |= SUPERVISOR_CB_LANDING_TIMEOUT;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_LANDING_TIMEOUT;
    }

    supervisorLog.info = supervisorGetInfoBitfield();
    supervisorLog.accNorm = sqrtf(supervisorSensors.acc.x * supervisorSensors.acc.x +
                                  supervisorSensors.acc.y * supervisorSensors.acc.y +
                                  supervisorSensors.acc.z * supervisorSensors.acc.z);
}

/* -------------------------------------------------------------------------
 * Estimator and commander arbitration
 * ---------------------------------------------------------------------- */
#define ESTIMATOR_FIFO_CAPACITY 16U
static EstimatorMeasurement estimatorFifo[ESTIMATOR_FIFO_CAPACITY];
static uint8_t estimatorHead = 0U;
static uint8_t estimatorTail = 0U;
static uint8_t estimatorCount = 0U;

bool estimatorEnqueue(const EstimatorMeasurement *measurement)
{
    if (!measurement) return false;
    if (estimatorCount >= ESTIMATOR_FIFO_CAPACITY) return false;

    estimatorFifo[estimatorTail] = *measurement;
    estimatorTail = (estimatorTail + 1U) % ESTIMATOR_FIFO_CAPACITY;
    estimatorCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement)
{
    if (!measurement) return false;
    if (estimatorCount == 0U) return false;

    *measurement = estimatorFifo[estimatorHead];
    estimatorHead = (estimatorHead + 1U) % ESTIMATOR_FIFO_CAPACITY;
    estimatorCount--;
    return true;
}

static float lastGyroEstimator[3] = {0};
static float lastAccEstimator[3] = {0};
static float lastBaroEstimator[3] = {0};
static float lastTofEstimator[3] = {0};
static float estVerticalVelocity = 0.0f;
static float estPositionZ = 0.0f;

void estimatorComplementary(uint32_t stabilizerStep)
{
    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) {
        switch (m.type) {
            case MeasurementTypeGyroscope:
                memcpy(lastGyroEstimator, m.data, sizeof(lastGyroEstimator));
                break;
            case MeasurementTypeAcceleration:
                memcpy(lastAccEstimator, m.data, sizeof(lastAccEstimator));
                break;
            case MeasurementTypeBarometer:
                memcpy(lastBaroEstimator, m.data, sizeof(lastBaroEstimator));
                break;
            case MeasurementTypeTOF:
                memcpy(lastTofEstimator, m.data, sizeof(lastTofEstimator));
                break;
        }
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        sensfusion6UpdateQ(lastGyroEstimator[0], lastGyroEstimator[1], lastGyroEstimator[2],
                           lastAccEstimator[0], lastAccEstimator[1], lastAccEstimator[2],
                           1.0f / 250.0f);
        sensfusion6GetEulerRPY(&stateEstimate.roll, &stateEstimate.pitch, &stateEstimate.yaw);
        sensfusion6GetQuaternion(&stateEstimate.qw, &stateEstimate.qx, &stateEstimate.qy, &stateEstimate.qz);

        sensfusion6Log.qw = qw; sensfusion6Log.qx = qx; sensfusion6Log.qy = qy; sensfusion6Log.qz = qz;
        sensfusion6Log.gravityX = gravityX; sensfusion6Log.gravityY = gravityY; sensfusion6Log.gravityZ = gravityZ;
        sensfusion6Log.accZbase = baseZacc;
        sensfusion6Log.isInit = sensfusion6IsInit;
        sensfusion6Log.isCalibrated = sensfusion6IsCalibrated;

        gyro.x = lastGyroEstimator[0]; gyro.y = lastGyroEstimator[1]; gyro.z = lastGyroEstimator[2];
        acc.x = lastAccEstimator[0]; acc.y = lastAccEstimator[1]; acc.z = lastAccEstimator[2];
        baro.asl = lastBaroEstimator[0]; baro.temp = lastBaroEstimator[1]; baro.pressure = lastBaroEstimator[2];

        float zAcc = sensfusion6GetAccZWithoutGravity(lastAccEstimator[0],
                                                      lastAccEstimator[1],
                                                      lastAccEstimator[2]);
        estVerticalVelocity += zAcc * 9.81f * (1.0f / 250.0f);
    }

    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
        estPositionZ += estVerticalVelocity * (1.0f / 100.0f);
    }
}

static Setpoint commanderActiveSetpoint;
static int commanderActivePriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t commanderLastUpdateTick = 0U;

bool commanderSetSetpoint(const Setpoint *setpoint, int priority)
{
    if (!setpoint) return false;

    if (priority == COMMANDER_PRIORITY_DISABLE) {
        commanderActiveSetpoint = *setpoint;
        commanderActivePriority = priority;
        commanderLastUpdateTick = supervisorCurrentTickMs;
        return true;
    }

    if (priority >= commanderActivePriority) {
        commanderActiveSetpoint = *setpoint;
        commanderActivePriority = priority;
        commanderLastUpdateTick = supervisorCurrentTickMs;

        if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
            trajectoryDisabled = true;
        }
        return true;
    }

    return false;
}

void commanderRelaxPriority(void)
{
    commanderActivePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void)
{
    if (supervisorCurrentTickMs < commanderLastUpdateTick) return 0U;
    return supervisorCurrentTickMs - commanderLastUpdateTick;
}

int commanderGetActivePriority(void)
{
    return commanderActivePriority;
}

static bool commanderGetSetpointInternal(Setpoint *out)
{
    if (!out) return false;
    *out = commanderActiveSetpoint;
    return true;
}

/* -------------------------------------------------------------------------
 * Stabilizer, compressed state, rate supervisor
 * ---------------------------------------------------------------------- */
static bool stabilizerInitialized = false;
static uint32_t stabilizerLoopStep = 0U;
static SensorData stabilizerSensors = {0};
static State stabilizerState = {0};
static Setpoint pendingHighLevelSetpoint;
static bool pendingHighLevelSetpointValid = false;

static float stabilizerBatteryVoltage = 4.2f;
static float stabilizerFilteredBattery = 4.2f;
static float stabilizerNominalVoltage = 4.2f;

static void sensorsInit(void) { sensfusion6Init(); }
static void stateEstimatorInit(void) { memset(&stabilizerState, 0, sizeof(stabilizerState)); }
static void controllerInit(void) { attitudeControllerInit(1.0f / ATTITUDE_RATE_HZ); }
static void powerDistributionInit(void) { }
static void motorsInit(void) { }
static void collisionAvoidanceInit(void) { }

static void sensorsWaitDataReady(void) { }
static void sensorsAcquire(void) { }
static void collisionAvoidanceUpdateSetpoint(Setpoint *s) { (void)s; }

static uint32_t quatcompress(float w, float x, float y, float z)
{
    int32_t c0 = (int32_t)(w * 10000.0f);
    int32_t c1 = (int32_t)(x * 10000.0f);
    int32_t c2 = (int32_t)(y * 10000.0f);
    int32_t c3 = (int32_t)(z * 10000.0f);

    uint16_t s0 = (uint16_t)c0;
    uint16_t s1 = (uint16_t)c1;
    uint16_t s2 = (uint16_t)c2;
    uint16_t s3 = (uint16_t)c3;

    return ((uint32_t)s0 << 24) | ((uint32_t)s1 << 16) |
           ((uint32_t)s2 << 8) | (uint32_t)s3;
}

void stabilizerInit(void)
{
    if (stabilizerInitialized) return;
    sensorsInit();
    stateEstimatorInit();
    controllerInit();
    powerDistributionInit();
    motorsInit();
    collisionAvoidanceInit();
    stabilizerInitialized = true;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint)
{
    if (!setpoint) return false;
    pendingHighLevelSetpoint = *setpoint;
    pendingHighLevelSetpointValid = true;
    return true;
}

void stabilizerTask(void)
{
    if (!stabilizerInitialized) stabilizerInit();

    if (healthShallWeRunTest()) {
        healthRunTests(&stabilizerSensors);
        return;
    }

    if (pendingHighLevelSetpointValid && supervisorCanFly()) {
        commanderSetSetpoint(&pendingHighLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
        pendingHighLevelSetpointValid = false;
    }

    sensorsWaitDataReady();
    sensorsAcquire();
    estimatorComplementary(stabilizerLoopStep);

    Setpoint sp;
    commanderGetSetpointInternal(&sp);

    supervisorUpdate(stabilizerLoopStep);
    collisionAvoidanceUpdateSetpoint(&sp);
    supervisorOverrideSetpoint(&sp, supervisorConditionBits, supervisorState);

    ControlData control;
    memset(&control, 0, sizeof(control));
    controllerPid(&stabilizerSensors, &sp, &stabilizerState, &control,
                  0.0f, 1.0f / ATTITUDE_RATE_HZ);

    MotorPower mp;
    powerDistribution(&control, &mp);

    int32_t motorPwms[4] = { mp.m1, mp.m2, mp.m3, mp.m4 };

    float filteredBattery = batteryCompensation(stabilizerBatteryVoltage,
                                                stabilizerFilteredBattery, 0.01f);
    stabilizerFilteredBattery = filteredBattery;
    for (int i = 0; i < 4; i++) {
        motorPwms[i] = motorsCompensateBatteryVoltage((uint16_t)motorPwms[i],
                                                      stabilizerNominalVoltage,
                                                      filteredBattery);
    }

    PowerCapResult cap = powerDistributionCap(motorPwms, 65535, 0);

    if (!supervisorCanFly() || !supervisorAreMotorsAllowedToRun()) {
        memset(motorPwms, 0, sizeof(motorPwms));
        cap.isCapped = false;
        cap.reduction = 0;
    }

    motor.m1req = (uint16_t)motorPwms[0];
    motor.m2req = (uint16_t)motorPwms[1];
    motor.m3req = (uint16_t)motorPwms[2];
    motor.m4req = (uint16_t)motorPwms[3];

    stabilizerLoopStep++;
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output)
{
    if (!state || !sensors || !output) return;

    output->position_mm[0] = (int32_t)(state->position.x * 1000.0f);
    output->position_mm[1] = (int32_t)(state->position.y * 1000.0f);
    output->position_mm[2] = (int32_t)(state->position.z * 1000.0f);

    output->velocity_mms[0] = (int32_t)(state->velocity.x * 1000.0f);
    output->velocity_mms[1] = (int32_t)(state->velocity.y * 1000.0f);
    output->velocity_mms[2] = (int32_t)(state->velocity.z * 1000.0f);

    output->acceleration_mms2[0] = (int32_t)(state->acc.x * 9810.0f);
    output->acceleration_mms2[1] = (int32_t)(state->acc.y * 9810.0f);
    output->acceleration_mms2[2] = (int32_t)((state->acc.z + 1.0f) * 9810.0f);

    output->gyro_millirad_s[0] = sensors->gyro.x * (M_PI / 180.0f) * 1000.0f;
    output->gyro_millirad_s[1] = -sensors->gyro.y * (M_PI / 180.0f) * 1000.0f;
    output->gyro_millirad_s[2] = sensors->gyro.z * (M_PI / 180.0f) * 1000.0f;

    output->quatCompressed = quatcompress(state->attitudeQuaternion.w,
                                          state->attitudeQuaternion.x,
                                          state->attitudeQuaternion.y,
                                          state->attitudeQuaternion.z);
}

bool rateSupervisorValidate(uint32_t measuredRate)
{
    return measuredRate >= 997U && measuredRate <= 1003U;
}

static uint32_t rateSupervisorTick = 0U;
static uint32_t rateSupervisorWaitStart = 0U;

void rateSupervisorTask(void)
{
    if (rateSupervisorTick == 0U) {
        rateSupervisorWaitStart = 0U;
    }

    if ((rateSupervisorTick - rateSupervisorWaitStart) < 2000U) {
        rateSupervisorTick++;
        return;
    }

    if (supervisorSensorsActive) {
        supervisorConditionBits |= SUPERVISOR_CB_DECK_FAULT;
        supervisorState = supervisorStateReset;
    }

    rateSupervisorWaitStart = rateSupervisorTick;
    rateSupervisorTick++;
}

/* -------------------------------------------------------------------------
 * Health
 * ---------------------------------------------------------------------- */
TestState healthTestState = testDone;
uint8_t motorPass = 0U;
uint8_t batteryPass = 0U;
float batterySag = 0.0f;

static bool propTestRequested = false;
static bool batteryTestRequested = false;

static float noiseVariance = 0.0f;
static float propVibration[4] = {0};
static float propVoltage[4] = {0};
static uint8_t currentPropMotor = 0U;
static uint8_t healthSampleCount = 0U;
static float healthBatteryVoltage = 4.2f;
static float healthIdleVoltage = 0.0f;
static float healthMinLoadedVoltage = 0.0f;
static float healthSampleBuffer[PROPTEST_NBR_OF_VARIANCE_VALUES];
static uint32_t healthBatteryTick = 0U;
static uint32_t healthRestartTick = 0U;
static uint32_t healthMotorFailureCount = 0U;
static bool healthAccelMode = false;
static bool healthMotorsStopped = true;
static int32_t healthRunningMotor = -1;
static float healthBatterySagThreshold = 1.0f;

void healthRequestPropTest(void)
{
    propTestRequested = true;
}

void healthRequestBatteryTest(void)
{
    batteryTestRequested = true;
}

bool healthShallWeRunTest(void)
{
    if (propTestRequested) {
        propTestRequested = false;
        healthTestState = configureAcc;
        motorPass = 0U;
        batteryPass = 0U;
        batterySag = 0.0f;
        healthMotorFailureCount = 0U;
        currentPropMotor = 0U;
        healthSampleCount = 0U;
        return true;
    }
    if (batteryTestRequested) {
        batteryTestRequested = false;
        healthTestState = testBattery;
        healthBatteryTick = 0U;
        healthIdleVoltage = 0.0f;
        healthMinLoadedVoltage = 1e9f;
        healthMotorsStopped = true;
        return true;
    }
    return healthTestState != testDone;
}

float variance(const float *buffer, int length)
{
    if (!buffer || length <= 0) return 0.0f;
    float sum = 0.0f;
    float sumSq = 0.0f;
    for (int i = 0; i < length; i++) {
        sum += buffer[i];
        sumSq += buffer[i] * buffer[i];
    }
    return sumSq - (sum * sum / (float)length);
}

static void healthStartCollection(void)
{
    healthSampleCount = 0U;
}

static void healthCollect(float value)
{
    if (healthSampleCount < PROPTEST_NBR_OF_VARIANCE_VALUES) {
        healthSampleBuffer[healthSampleCount++] = value;
    }
}

static float healthCollectedVariance(void)
{
    return variance(healthSampleBuffer, healthSampleCount);
}

static float healthBatteryVoltageFromSensor(const SensorData *sensorData)
{
    float v = sensorData->tofRange;
    if (v <= 0.0f) v = sensorData->baroAsl;
    if (v <= 0.0f) v = 4.2f;
    return v;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIndex)
{
    if (highThreshold == 0.0f) return true;
    if (measuredValue >= lowThreshold && measuredValue <= highThreshold) {
        motorPass |= (uint8_t)(1U << motorIndex);
        return true;
    }
    healthMotorFailureCount++;
    return false;
}

void healthRunTests(const SensorData *sensorData)
{
    if (!sensorData) return;

    healthBatteryVoltage = healthBatteryVoltageFromSensor(sensorData);

    switch (healthTestState) {
        case configureAcc:
            healthAccelMode = true;
            healthMotorsStopped = true;
            healthRunningMotor = -1;
            healthIdleVoltage = healthBatteryVoltage;
            healthMinLoadedVoltage = healthBatteryVoltage;
            healthStartCollection();
            healthTestState = measureNoiseFloor;
            break;

        case measureNoiseFloor: {
            float mag = sqrtf(sensorData->acc.x * sensorData->acc.x +
                              sensorData->acc.y * sensorData->acc.y +
                              sensorData->acc.z * sensorData->acc.z);
            healthCollect(mag);
            if (healthSampleCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
                noiseVariance = healthCollectedVariance();
                currentPropMotor = 0U;
                healthStartCollection();
                healthRunningMotor = -1;
                healthMotorsStopped = false;
                healthTestState = measureProp;
            }
            break;
        }

        case measureProp: {
            if (currentPropMotor < 4U) {
                if (healthSampleCount == 0U && healthRunningMotor < 0) {
                    healthRunningMotor = currentPropMotor;
                    healthMotorsStopped = false;
                }

                float mag = sqrtf(sensorData->acc.x * sensorData->acc.x +
                                  sensorData->acc.y * sensorData->acc.y +
                                  sensorData->acc.z * sensorData->acc.z);
                healthCollect(mag);

                if (healthSampleCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
                    propVibration[currentPropMotor] = healthCollectedVariance();
                    propVoltage[currentPropMotor] = healthBatteryVoltage;
                    currentPropMotor++;
                    healthRunningMotor = -1;
                    healthMotorsStopped = true;
                    healthStartCollection();
                }
            } else {
                healthRunningMotor = -1;
                healthMotorsStopped = true;
                healthTestState = evaluatePropResult;
            }
            break;
        }

        case evaluatePropResult:
            for (uint8_t i = 0; i < 4U; i++) {
                (void)evaluatePropTest(0.0f, 100.0f, propVibration[i], i);
            }
            healthAccelMode = false;
            healthMotorsStopped = true;
            healthTestState = testDone;
            break;

        case testBattery:
            if (healthBatteryTick == 0U) {
                healthIdleVoltage = healthBatteryVoltage;
                healthMinLoadedVoltage = 1e9f;
            }
            healthBatteryTick++;
            if (healthBatteryTick == 1U) {
                healthMotorsStopped = false;
            } else if (healthBatteryTick >= 2U && healthBatteryTick < 50U) {
                float v = healthBatteryVoltage;
                if (v < healthMinLoadedVoltage) healthMinLoadedVoltage = v;
            } else if (healthBatteryTick >= 50U) {
                batterySag = healthIdleVoltage - healthMinLoadedVoltage;
                batteryPass = (batterySag < healthBatterySagThreshold) ? 1U : 0U;
                healthMotorsStopped = true;
                healthTestState = evaluateBatResult;
            }
            break;

        case evaluateBatResult:
            healthTestState = testDone;
            break;

        case restartBatTest:
            healthRestartTick++;
            if (healthRestartTick >= 2000U) {
                healthRestartTick = 0U;
                healthTestState = testBattery;
                healthBatteryTick = 0U;
                healthIdleVoltage = 0.0f;
                healthMinLoadedVoltage = 1e9f;
            }
            break;

        case testDone:
        default:
            break;
    }

    healthLog.motorPass = motorPass;
    healthLog.batteryPass = batteryPass;
    healthLog.batterySag = batterySag;
    healthLog.motorTestCount = healthMotorFailureCount;
}

/* -------------------------------------------------------------------------
 * CRTP transport
 * ---------------------------------------------------------------------- */
typedef struct {
    CrtpPacket packets[CRTP_TX_QUEUE_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} TxQueue;

typedef struct {
    CrtpPacket packets[CRTP_RX_QUEUE_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} RxQueue;

static TxQueue crtpTxQueue;
static RxQueue crtpRxQueues[CRTP_NBR_OF_PORTS];
static bool crtpRxQueueCreated[CRTP_NBR_OF_PORTS] = {false};
static CrtpPortCallback crtpPortCallbacks[CRTP_NBR_OF_PORTS] = {NULL};
static bool crtpError = false;
static CrtpLink *currentCrtpLink = NULL;
static uint32_t crtpRxCount = 0U;
static uint32_t crtpTxCount = 0U;
static uint32_t crtpRxRate = 0U;
static uint32_t crtpTxRate = 0U;
static uint32_t statsLastTick = 0U;
static bool statsLastTickValid = false;

static uint32_t crtpTxTaskTick = 0U;
static bool crtpTxRetryPending = false;
static uint32_t crtpTxRetryTick = 0U;

static bool nopSendPacket(CrtpPacket *packet) { (void)packet; return false; }
static bool nopReceivePacket(CrtpPacket *packet) { (void)packet; return false; }
static bool nopIsConnected(void) { return true; }
static void nopSetEnable(bool enable) { (void)enable; }
static void nopReset(void) { }

static CrtpLink nopLinkImpl = {
    nopSendPacket,
    nopReceivePacket,
    nopIsConnected,
    nopSetEnable,
    nopReset
};

void crtpInit(void)
{
    static bool initialized = false;
    if (initialized) return;

    memset(&crtpTxQueue, 0, sizeof(crtpTxQueue));
    for (int i = 0; i < CRTP_NBR_OF_PORTS; i++) {
        memset(&crtpRxQueues[i], 0, sizeof(crtpRxQueues[i]));
        crtpRxQueueCreated[i] = false;
        crtpPortCallbacks[i] = NULL;
    }
    crtpError = false;
    currentCrtpLink = &nopLinkImpl;
    crtpRxCount = 0U;
    crtpTxCount = 0U;
    crtpRxRate = 0U;
    crtpTxRate = 0U;
    statsLastTickValid = false;
    crtpTxTaskTick = 0U;
    crtpTxRetryPending = false;
    crtpTxRetryTick = 0U;
    initialized = true;
}

void crtpInitTaskQueue(uint8_t port)
{
    if (port >= CRTP_NBR_OF_PORTS) {
        crtpError = true;
        return;
    }
    if (crtpRxQueueCreated[port]) {
        crtpError = true;
        return;
    }
    memset(&crtpRxQueues[port], 0, sizeof(crtpRxQueues[port]));
    crtpRxQueueCreated[port] = true;
}

bool crtpSendPacket(const CrtpPacket *packet)
{
    if (!packet || packet->size > sizeof(packet->data)) return false;
    if (crtpTxQueue.count >= CRTP_TX_QUEUE_SIZE) return false;

    crtpTxQueue.packets[crtpTxQueue.tail] = *packet;
    crtpTxQueue.tail = (crtpTxQueue.tail + 1U) % CRTP_TX_QUEUE_SIZE;
    crtpTxQueue.count++;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet)
{
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet)
{
    if (port >= CRTP_NBR_OF_PORTS || !crtpRxQueueCreated[port] || !packet) return false;
    if (crtpRxQueues[port].count == 0U) return false;

    *packet = crtpRxQueues[port].packets[crtpRxQueues[port].head];
    crtpRxQueues[port].head = (crtpRxQueues[port].head + 1U) % CRTP_RX_QUEUE_SIZE;
    crtpRxQueues[port].count--;
    return true;
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet)
{
    return crtpReceivePacket(port, packet);
}

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet, uint32_t wait_ms)
{
    if (wait_ms == 0U) return crtpReceivePacket(port, packet);

    for (uint32_t i = 0U; i <= wait_ms; i++) {
        if (crtpReceivePacket(port, packet)) return true;
    }
    return false;
}

void crtpRxTask(void)
{
    if (!currentCrtpLink || currentCrtpLink == &nopLinkImpl) return;
    if (!currentCrtpLink->receivePacket) return;

    CrtpPacket packet;
    while (currentCrtpLink->receivePacket(&packet)) {
        if (packet.port < CRTP_NBR_OF_PORTS) {
            if (crtpRxQueueCreated[packet.port] &&
                crtpRxQueues[packet.port].count < CRTP_RX_QUEUE_SIZE) {
                crtpRxQueues[packet.port].packets[crtpRxQueues[packet.port].tail] = packet;
                crtpRxQueues[packet.port].tail = (crtpRxQueues[packet.port].tail + 1U) % CRTP_RX_QUEUE_SIZE;
                crtpRxQueues[packet.port].count++;
            }
            if (crtpPortCallbacks[packet.port]) {
                crtpPortCallbacks[packet.port](&packet);
            }
        }
        crtpRxCount++;
    }
}

void crtpTxTask(void)
{
    if (!currentCrtpLink || currentCrtpLink == &nopLinkImpl) return;
    if (!currentCrtpLink->sendPacket) return;
    if (crtpTxQueue.count == 0U) {
        crtpTxRetryPending = false;
        return;
    }

    crtpTxTaskTick++;
    if (crtpTxRetryPending && (crtpTxTaskTick - crtpTxRetryTick) < 10U) {
        return;
    }

    CrtpPacket packet = crtpTxQueue.packets[crtpTxQueue.head];
    if (currentCrtpLink->sendPacket(&packet)) {
        crtpTxQueue.head = (crtpTxQueue.head + 1U) % CRTP_TX_QUEUE_SIZE;
        crtpTxQueue.count--;
        crtpTxCount++;
        crtpTxRetryPending = false;
    } else {
        crtpTxRetryPending = true;
        crtpTxRetryTick = crtpTxTaskTick;
    }
}

void crtpSetLink(CrtpLink *newLink)
{
    if (currentCrtpLink && currentCrtpLink->setEnable) {
        currentCrtpLink->setEnable(false);
    }
    currentCrtpLink = newLink ? newLink : &nopLinkImpl;
    if (currentCrtpLink->setEnable) {
        currentCrtpLink->setEnable(true);
    }
}

void crtpReset(void)
{
    crtpTxQueue.head = 0U;
    crtpTxQueue.tail = 0U;
    crtpTxQueue.count = 0U;
    crtpTxRetryPending = false;
    crtpTxRetryTick = 0U;
    if (currentCrtpLink && currentCrtpLink->reset) {
        currentCrtpLink->reset();
    }
}

bool crtpIsConnected(void)
{
    if (currentCrtpLink && currentCrtpLink->isConnected) {
        return currentCrtpLink->isConnected();
    }
    return true;
}

uint32_t crtpGetFreeTxQueuePackets(void)
{
    return (uint32_t)(CRTP_TX_QUEUE_SIZE - crtpTxQueue.count);
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback)
{
    if (port >= CRTP_NBR_OF_PORTS) return;
    crtpPortCallbacks[port] = callback;
}

void updateStats(void)
{
    uint32_t now = supervisorCurrentTickMs;
    if (!statsLastTickValid) {
        statsLastTick = now;
        statsLastTickValid = true;
        return;
    }

    if (now < statsLastTick) {
        statsLastTick = now;
        return;
    }

    uint32_t elapsed = now - statsLastTick;
    if (elapsed >= 500U && elapsed > 0U) {
        crtpRxRate = crtpRxCount * 1000U / elapsed;
        crtpTxRate = crtpTxCount * 1000U / elapsed;
        crtpRxCount = 0U;
        crtpTxCount = 0U;
        statsLastTick = now;
    }
}

/* -------------------------------------------------------------------------
 * Deck discovery and log objects
 * ---------------------------------------------------------------------- */
uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity)
{
    if (!decks || capacity == 0U) return 0U;

    static const uint8_t knownI2c[] = {0x20U, 0x21U, 0x22U};
    static const uint64_t knownOneWire[] = {0x11ULL, 0x22ULL, 0x33ULL};
    static uint8_t i2cCount = (uint8_t)(sizeof(knownI2c) / sizeof(knownI2c[0]));
    static uint8_t oneWireCount = (uint8_t)(sizeof(knownOneWire) / sizeof(knownOneWire[0]));

    uint8_t written = 0U;

    for (uint8_t i = 0; i < i2cCount && written < capacity; i++) {
        bool duplicate = false;
        for (uint8_t j = 0; j < written; j++) {
            if (decks[j].foundByI2C && decks[j].i2cAddress == knownI2c[i]) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            decks[written].foundByI2C = true;
            decks[written].foundByOneWire = false;
            decks[written].i2cAddress = knownI2c[i];
            decks[written].oneWireRomId = 0U;
            written++;
        }
    }

    for (uint8_t i = 0; i < oneWireCount && written < capacity; i++) {
        bool duplicate = false;
        for (uint8_t j = 0; j < written; j++) {
            if (decks[j].foundByOneWire && decks[j].oneWireRomId == knownOneWire[i]) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            decks[written].foundByI2C = false;
            decks[written].foundByOneWire = true;
            decks[written].i2cAddress = 0U;
            decks[written].oneWireRomId = knownOneWire[i];
            written++;
        }
    }

    return written;
}

StateEstimateLog stateEstimate = {0};
Axis3Log gyro = {0};
Axis3Log acc = {0};
BaroLog baro = {0};
MotorLog motor = {0};
Sensfusion6Log sensfusion6Log = {0};
SupervisorLog supervisorLog = {0};
HealthLog healthLog = {0};
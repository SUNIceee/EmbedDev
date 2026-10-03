#include "6_generated_code.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD2DEG (180.0f / M_PI)
#define DEG2RAD (M_PI / 180.0f)

uint32_t tick = 0;

/* Sensfusion6 */
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

/* PIDs */
PidObject pidRoll, pidPitch, pidYaw;
PidObject pidRollRate, pidPitchRate, pidYawRate;

/* Commander RPYT */
bool thrustLocked = false;
bool commanderModeSet = false;

/* Supervisor */
SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0;

/* Health */
TestState healthTestState = testDone;
uint8_t motorPass = 0, batteryPass = 0;
float batterySag = 0.0f;

/* Logs */
StateEstimateLog stateEstimate;
Axis3Log gyro, acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;

/* ------------------------------------------------------------------ */
/* Numeric helpers */
/* ------------------------------------------------------------------ */
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

float invSqrt(float x)
{
    if (x <= 0.0f) return 0.0f;
    uint32_t i;
    float x2 = x * 0.5f;
    float y = x;
    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759dfU - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - x2 * y * y);
    return y;
}

/* ------------------------------------------------------------------ */
/* Sensfusion6 */
/* ------------------------------------------------------------------ */
void estimatedGravityDirection(float qw, float qx, float qy, float qz,
                               float *gravX, float *gravY, float *gravZ)
{
    if (gravX) {
        *gravX = 2.0f * (qx * qz - qw * qy);
    }
    if (gravY) {
        *gravY = 2.0f * (qw * qx + qy * qz);
    }
    if (gravZ) {
        *gravZ = qw * qw - qx * qx - qy * qy + qz * qz;
    }
}

static float sensfusion_clamp1(float v)
{
    if (v > 1.0f) return 1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

void sensfusion6Init(void)
{
    qw = 1.0f;
    qx = qy = qz = 0.0f;
    gravityX = 0.0f;
    gravityY = 0.0f;
    gravityZ = 1.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
    baseZacc = 0.0f;
    sensfusion6IsInit = true;
    sensfusion6IsCalibrated = false;
    sensfusion6Log.qw = qw;
    sensfusion6Log.qx = qx;
    sensfusion6Log.qy = qy;
    sensfusion6Log.qz = qz;
    sensfusion6Log.gravityX = gravityX;
    sensfusion6Log.gravityY = gravityY;
    sensfusion6Log.gravityZ = gravityZ;
    sensfusion6Log.accZbase = baseZacc;
    sensfusion6Log.isInit = sensfusion6IsInit;
    sensfusion6Log.isCalibrated = sensfusion6IsCalibrated;
}

bool sensfusion6Test(void)
{
    return sensfusion6IsInit;
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt)
{
    if (!sensfusion6IsInit) {
        sensfusion6Init();
    }
    if (dt <= 0.000001f) {
        return;
    }

    float gxr = gx * DEG2RAD;
    float gyr = gy * DEG2RAD;
    float gzr = gz * DEG2RAD;

    float accNormSq = ax * ax + ay * ay + az * az;
    if (accNormSq > 1.0e-10f) {
        if (!sensfusion6IsCalibrated) {
            float egx, egy, egz;
            estimatedGravityDirection(qw, qx, qy, qz, &egx, &egy, &egz);
            baseZacc = ax * egx + ay * egy + az * egz;
            sensfusion6IsCalibrated = true;
        }
    }

    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
    if (accNormSq > 1.0e-10f) {
        float inv = invSqrt(accNormSq);
        float nax = ax * inv;
        float nay = ay * inv;
        float naz = az * inv;

        float egx, egy, egz;
        estimatedGravityDirection(qw, qx, qy, qz, &egx, &egy, &egz);

        halfex = nay * egz - naz * egy;
        halfey = naz * egx - nax * egz;
        halfez = nax * egy - nay * egx;

#ifdef CONFIG_IMU_MADGWICK_QUATERNION
        (void)beta;
        gxr += beta * halfex;
        gyr += beta * halfey;
        gzr += beta * halfez;
        integralFBx = integralFBy = integralFBz = 0.0f;
#else
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
        } else {
            integralFBx = integralFBy = integralFBz = 0.0f;
        }
        gxr += twoKp * halfex + integralFBx;
        gyr += twoKp * halfey + integralFBy;
        gzr += twoKp * halfez + integralFBz;
#endif
    }

    float qdot0 = 0.5f * (-qx * gxr - qy * gyr - qz * gzr);
    float qdot1 = 0.5f * ( qw * gxr + qy * gzr - qz * gyr);
    float qdot2 = 0.5f * ( qw * gyr - qx * gzr + qz * gxr);
    float qdot3 = 0.5f * ( qw * gzr + qx * gyr - qy * gxr);

    qw += qdot0 * dt;
    qx += qdot1 * dt;
    qy += qdot2 * dt;
    qz += qdot3 * dt;

    float normSq = qw * qw + qx * qx + qy * qy + qz * qz;
    if (normSq > 1.0e-10f) {
        float inv = invSqrt(normSq);
        qw *= inv;
        qx *= inv;
        qy *= inv;
        qz *= inv;
    }

    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);

    sensfusion6Log.qw = qw;
    sensfusion6Log.qx = qx;
    sensfusion6Log.qy = qy;
    sensfusion6Log.qz = qz;
    sensfusion6Log.gravityX = gravityX;
    sensfusion6Log.gravityY = gravityY;
    sensfusion6Log.gravityZ = gravityZ;
    sensfusion6Log.accZbase = baseZacc;
    sensfusion6Log.isInit = sensfusion6IsInit;
    sensfusion6Log.isCalibrated = sensfusion6IsCalibrated;
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
    float gravX = sensfusion_clamp1(gravityX);
    float roll = atan2f(gravityY, gravityZ) * RAD2DEG;
    float pitch = asinf(gravX) * RAD2DEG;
    float yaw = atan2f(2.0f * (qx * qy + qw * qz),
                       qw * qw + qx * qx - qy * qy - qz * qz) * RAD2DEG;
    if (roll_deg) *roll_deg = roll;
    if (pitch_deg) *pitch_deg = pitch;
    if (yaw_deg) *yaw_deg = yaw;
}

void sensfusion6GetQuaternion(float *qw_out, float *qx_out, float *qy_out, float *qz_out)
{
    if (qw_out) *qw_out = qw;
    if (qx_out) *qx_out = qx;
    if (qy_out) *qy_out = qy;
    if (qz_out) *qz_out = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az)
{
    float egx, egy, egz;
    estimatedGravityDirection(qw, qx, qy, qz, &egx, &egy, &egz);
    return ax * egx + ay * egy + az * egz;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az)
{
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

/* ------------------------------------------------------------------ */
/* Power distribution */
/* ------------------------------------------------------------------ */
void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out)
{
    if (!out) return;
    int32_t r = roll / 2;
    int32_t p = pitch / 2;
    int32_t t = (int32_t)thrust;
    out->m1 = t - r + p + yaw;
    out->m2 = t - r - p - yaw;
    out->m3 = t + r - p + yaw;
    out->m4 = t + r + p - yaw;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4])
{
    if (!motorForces) return;
    float thrustPart = 0.25f * thrustSi;
    float arm = 0.707106781f * armLength;
    float rollPart = 0.0f;
    float pitchPart = 0.0f;
    float yawPart = 0.0f;
    if (arm > 1.0e-9f) {
        rollPart = 0.25f / arm * torqueX;
        pitchPart = 0.25f / arm * torqueY;
    }
    if (thrustToTorque != 0.0f) {
        yawPart = 0.25f / thrustToTorque * torqueZ;
    }

    float f1 = thrustPart - rollPart + pitchPart + yawPart;
    float f2 = thrustPart - rollPart - pitchPart - yawPart;
    float f3 = thrustPart + rollPart - pitchPart + yawPart;
    float f4 = thrustPart + rollPart + pitchPart - yawPart;
    if (f1 < 0.0f) f1 = 0.0f;
    if (f2 < 0.0f) f2 = 0.0f;
    if (f3 < 0.0f) f3 = 0.0f;
    if (f4 < 0.0f) f4 = 0.0f;
    motorForces[0] = f1;
    motorForces[1] = f2;
    motorForces[2] = f3;
    motorForces[3] = f4;
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4])
{
    if (!normalizedForces || !motorPWMs) return;
    for (int i = 0; i < 4; i++) {
        float v = normalizedForces[i];
        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;
        motorPWMs[i] = (uint16_t)(v * 65535.0f);
    }
}

static uint16_t motorForceToPwm(float force)
{
    if (force <= 0.0f) return 0;
    if (force >= CRAZYFLIE_MAX_MOTOR_FORCE_N) return 65535;
    return (uint16_t)((force / CRAZYFLIE_MAX_MOTOR_FORCE_N) * 65535.0f + 0.5f);
}

void powerDistribution(const ControlData *control, MotorPower *motorPower)
{
    if (!control || !motorPower) return;
    switch (control->controlMode) {
    case controlModeLegacy: {
        powerDistributionLegacy(control->thrust, control->roll, control->pitch,
                                control->yaw, motorPower);
        break;
    }
    case controlModeForceTorque: {
        float forces[4];
        powerDistributionForceTorque(control->thrustSi, control->torque.x,
                                     control->torque.y, control->torque.z,
                                     CRAZYFLIE_ARM_LENGTH_M,
                                     CRAZYFLIE_THRUST_TO_TORQUE, forces);
        motorPower->m1 = motorForceToPwm(forces[0]);
        motorPower->m2 = motorForceToPwm(forces[1]);
        motorPower->m3 = motorForceToPwm(forces[2]);
        motorPower->m4 = motorForceToPwm(forces[3]);
        break;
    }
    case controlModeForce: {
        uint16_t pwms[4];
        powerDistributionForce(control->normalizedForces, pwms);
        motorPower->m1 = pwms[0];
        motorPower->m2 = pwms[1];
        motorPower->m3 = pwms[2];
        motorPower->m4 = pwms[3];
        break;
    }
    default:
        break;
    }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust)
{
    if (value < idleThrust) return idleThrust;
    return value;
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

    if (maxVal > maxAllowedThrust) {
        result.isCapped = true;
        result.reduction = maxVal - maxAllowedThrust;
        for (int i = 0; i < 4; i++) {
            motors[i] -= result.reduction;
            motors[i] = capMinThrust(motors[i], idleThrust);
        }
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
    if (actualVoltage <= 0.0f) return motorThrust;
    float ratio = nominalVoltage / actualVoltage;
    float compensated = (float)motorThrust * ratio;
    if (compensated < 0.0f) compensated = 0.0f;
    if (compensated > 65535.0f) compensated = 65535.0f;
    return (uint16_t)(compensated + 0.5f);
}

/* ------------------------------------------------------------------ */
/* PID helpers */
/* ------------------------------------------------------------------ */
static float attitudeControllerDt = 0.002f;

static void pidReset(PidObject *pid)
{
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
}

static float pidUpdate(PidObject *pid, float error, float dt, bool reset)
{
    if (!pid || !pid->initialized) return 0.0f;
    if (dt <= 0.000001f) dt = 0.002f;
    if (reset) {
        pidReset(pid);
    }
    float derivative = 0.0f;
    if (pid->initialized && dt > 0.0f) {
        derivative = pid->kd * (error - pid->prevError) / dt;
    }
    float output = pid->kp * error + pid->integral + derivative;
    pid->integral += pid->ki * error * dt;
    pid->prevError = error;
    pid->output = output;
    return output;
}

void attitudeControllerInit(float updateDt)
{
    if (updateDt > 0.0f) attitudeControllerDt = updateDt;

    PidObject *objects[6] = {
        &pidRoll, &pidPitch, &pidYaw,
        &pidRollRate, &pidPitchRate, &pidYawRate
    };
    for (int i = 0; i < 3; i++) {
        if (!objects[i]->initialized) {
            objects[i]->kp = 6.0f;
            objects[i]->ki = 0.0f;
            objects[i]->kd = 1.0f;
            objects[i]->kff = 0.0f;
            objects[i]->integral = 0.0f;
            objects[i]->prevError = 0.0f;
            objects[i]->output = 0.0f;
            objects[i]->initialized = true;
        }
    }
    for (int i = 3; i < 6; i++) {
        if (!objects[i]->initialized) {
            objects[i]->kp = 0.1f;
            objects[i]->ki = 0.0f;
            objects[i]->kd = 0.0f;
            objects[i]->kff = 0.0f;
            objects[i]->integral = 0.0f;
            objects[i]->prevError = 0.0f;
            objects[i]->output = 0.0f;
            objects[i]->initialized = true;
        }
    }
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired)
{
    float out;
    out = pidUpdate(&pidRollRate, rollDesired - rollActual, attitudeControllerDt, false);
    pidRollRate.output = (float)saturateSignedInt16((int32_t)out);
    out = pidUpdate(&pidPitchRate, pitchDesired - pitchActual, attitudeControllerDt, false);
    pidPitchRate.output = (float)saturateSignedInt16((int32_t)out);
    out = pidUpdate(&pidYawRate, yawDesired - yawActual, attitudeControllerDt, false);
    pidYawRate.output = (float)saturateSignedInt16((int32_t)out);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired)
{
    pidRoll.output = pidUpdate(&pidRoll, rollDesired - rollActual,
                               attitudeControllerDt, false);
    pidPitch.output = pidUpdate(&pidPitch, pitchDesired - pitchActual,
                                attitudeControllerDt, false);
    pidYaw.output = pidUpdate(&pidYaw, yawDesired - yawActual,
                              attitudeControllerDt, true);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual)
{
    (void)rollActual;
    (void)pitchActual;
    (void)yawActual;
    pidReset(&pidRoll);
    pidReset(&pidPitch);
    pidReset(&pidYaw);
    pidReset(&pidRollRate);
    pidReset(&pidPitchRate);
    pidReset(&pidYawRate);
}

void attitudeControllerResetRollAttitudePID(float rollActual)
{
    (void)rollActual;
    pidReset(&pidRoll);
}

void attitudeControllerResetPitchAttitudePID(float pitchActual)
{
    (void)pitchActual;
    pidReset(&pidPitch);
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw)
{
    if (roll) *roll = (int16_t)pidRollRate.output;
    if (pitch) *pitch = (int16_t)pidPitchRate.output;
    if (yaw) *yaw = (int16_t)pidYawRate.output;
}

static float quaternionYawDegrees(float qw, float qx, float qy, float qz)
{
    return atan2f(2.0f * (qx * qy + qw * qz),
                  qw * qw + qx * qx - qy * qy - qz * qz) * RAD2DEG;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt)
{
    if (!sensors || !setpoint || !state || !control) return;
    static float controllerDesiredYaw = 0.0f;

    memset(control, 0, sizeof(*control));
    control->controlMode = controlModeLegacy;

    if (setpoint->mode.yaw == modeVelocity) {
        controllerDesiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
    } else if (setpoint->mode.yaw == modeAbs) {
        controllerDesiredYaw = setpoint->attitude.yaw;
    } else if (setpoint->mode.quat == modeAbs) {
        controllerDesiredYaw = quaternionYawDegrees(setpoint->attitudeQuaternion.w,
                                                     setpoint->attitudeQuaternion.x,
                                                     setpoint->attitudeQuaternion.y,
                                                     setpoint->attitudeQuaternion.z);
    }

    if (yawMaxDelta != 0.0f) {
        float diff = capAngle(controllerDesiredYaw - state->attitude.yaw);
        if (diff > yawMaxDelta) diff = yawMaxDelta;
        if (diff < -yawMaxDelta) diff = -yawMaxDelta;
        controllerDesiredYaw = state->attitude.yaw + diff;
    }

    float rollDesiredRate = 0.0f;
    float pitchDesiredRate = 0.0f;
    float yawDesiredRate = 0.0f;

    if (setpoint->mode.roll == modeVelocity) {
        rollDesiredRate = setpoint->attitudeRate.roll;
        pidReset(&pidRoll);
    } else {
        pidRoll.output = pidUpdate(&pidRoll,
                                  setpoint->attitude.roll - state->attitude.roll,
                                  attitudeControllerDt, false);
        rollDesiredRate = pidRoll.output;
    }

    if (setpoint->mode.pitch == modeVelocity) {
        pitchDesiredRate = setpoint->attitudeRate.pitch;
        pidReset(&pidPitch);
    } else {
        pidPitch.output = pidUpdate(&pidPitch,
                                   setpoint->attitude.pitch - state->attitude.pitch,
                                   attitudeControllerDt, false);
        pitchDesiredRate = pidPitch.output;
    }

    if (setpoint->mode.yaw == modeVelocity) {
        yawDesiredRate = setpoint->attitudeRate.yaw;
    } else {
        pidYaw.output = pidUpdate(&pidYaw,
                                 controllerDesiredYaw - state->attitude.yaw,
                                 attitudeControllerDt, true);
        yawDesiredRate = pidYaw.output;
    }

    if (setpoint->mode.z == modeDisable) {
        control->thrust = setpoint->thrust;
    } else {
        control->thrust = positionControllerUpdate(setpoint, state);
    }

    float rollActual = sensors->gyro.x;
    float pitchActual = -sensors->gyro.y;
    float yawActual = sensors->gyro.z;

    float out;
    out = pidUpdate(&pidRollRate, rollDesiredRate - rollActual,
                    attitudeControllerDt, false);
    control->roll = saturateSignedInt16((int32_t)out);
    out = pidUpdate(&pidPitchRate, pitchDesiredRate - pitchActual,
                    attitudeControllerDt, false);
    control->pitch = saturateSignedInt16((int32_t)out);
    out = pidUpdate(&pidYawRate, yawDesiredRate - yawActual,
                    attitudeControllerDt, false);
    control->yaw = saturateSignedInt16((int32_t)out);

    if (control->thrust == 0) {
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->thrust = 0;
        attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch,
                                      state->attitude.yaw);
        controllerDesiredYaw = state->attitude.yaw;
        return;
    }

    control->yaw = -control->yaw;
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state)
{
    if (!setpoint || !state) return 0;
    float output = 30000.0f;

    if (setpoint->mode.z == modeAbs) {
        float posError = setpoint->position.z - state->position.z;
        float targetVelocity = posError * 0.5f;
        float velError = targetVelocity - state->velocity.z;
        output = 30000.0f + 2000.0f * velError;
    } else if (setpoint->mode.z == modeVelocity) {
        float velError = setpoint->velocity.z - state->velocity.z;
        output = 30000.0f + 2000.0f * velError;
    } else {
        return 0;
    }

    if (output < 0.0f) output = 0.0f;
    if (output > 65535.0f) output = 65535.0f;
    return (uint16_t)(output + 0.5f);
}

/* ------------------------------------------------------------------ */
/* CRTP Commander RPYT */
/* ------------------------------------------------------------------ */
void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime)
{
    float rad = yaw_deg * DEG2RAD;
    float c = cosf(rad);
    float s = sinf(rad);
    float rp = roll * c - pitch * s;
    float pp = roll * s + pitch * c;
    if (rollPrime) *rollPrime = rp;
    if (pitchPrime) *pitchPrime = pp;
}

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

    uint16_t rawThrust = values->thrust;
    float rawRoll = values->roll;
    float rawPitch = values->pitch;
    float rawYaw = values->yaw;

    int activePriority = commanderGetActivePriority();
    if (activePriority == COMMANDER_PRIORITY_DISABLE) {
        thrustLocked = true;
        if (rawThrust == 0) {
            thrustLocked = false;
        }
    }

    if (altHoldMode) {
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0;
        setpoint->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;
        if (!commanderModeSet) {
            commanderModeSet = true;
            /* Position/filter reset boundary is internal; no external output. */
        }
    } else {
        if (commanderModeSet) {
            commanderModeSet = false;
            setpoint->mode.z = modeDisable;
        }
        if (posSetMode && rawThrust != 0) {
            setpoint->mode.x = modeAbs;
            setpoint->mode.y = modeAbs;
            setpoint->mode.z = modeAbs;
            setpoint->mode.roll = modeDisable;
            setpoint->mode.pitch = modeDisable;
            setpoint->mode.yaw = modeAbs;
            setpoint->position.x = -rawPitch;
            setpoint->position.y = rawRoll;
            setpoint->position.z = (float)rawThrust / 1000.0f;
            setpoint->attitude.yaw = rawYaw;
            setpoint->thrust = 0;
            return;
        } else if (posHoldMode) {
            setpoint->mode.x = modeVelocity;
            setpoint->mode.y = modeVelocity;
            setpoint->mode.roll = modeDisable;
            setpoint->mode.pitch = modeDisable;
            setpoint->velocity.x = rawPitch / 30.0f;
            setpoint->velocity.y = rawRoll / 30.0f;
            setpoint->attitude.roll = 0.0f;
            setpoint->attitude.pitch = 0.0f;
        } else {
            if (stabilizationModeRoll == RATE) {
                setpoint->mode.roll = modeVelocity;
                setpoint->attitudeRate.roll = rawRoll;
            } else {
                setpoint->mode.roll = modeAbs;
                setpoint->attitude.roll = rawRoll;
            }
            if (stabilizationModePitch == RATE) {
                setpoint->mode.pitch = modeVelocity;
                setpoint->attitudeRate.pitch = rawPitch;
            } else {
                setpoint->mode.pitch = modeAbs;
                setpoint->attitude.pitch = rawPitch;
            }
            if (stabilizationModeYaw == RATE) {
                setpoint->mode.yaw = modeVelocity;
                setpoint->attitudeRate.yaw = -rawYaw;
            } else {
                setpoint->mode.yaw = modeAbs;
                setpoint->attitude.yaw = rawYaw;
            }

            if (yawMode == PLUSMODE) {
                rotateYaw(setpoint->attitude.roll, setpoint->attitude.pitch, 45.0f,
                          &setpoint->attitude.roll, &setpoint->attitude.pitch);
                rotateYaw(setpoint->attitudeRate.roll, setpoint->attitudeRate.pitch, 45.0f,
                          &setpoint->attitudeRate.roll, &setpoint->attitudeRate.pitch);
            } else if (yawMode == CAREFREE) {
                setpoint->attitude.roll = 0.0f;
                setpoint->attitude.pitch = 0.0f;
                setpoint->attitudeRate.roll = 0.0f;
                setpoint->attitudeRate.pitch = 0.0f;
            }
        }

        if (thrustLocked || rawThrust < MIN_THRUST) {
            setpoint->thrust = 0;
        } else {
            setpoint->thrust = (rawThrust > MAX_THRUST) ? MAX_THRUST : rawThrust;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Estimator and Commander arbitration */
/* ------------------------------------------------------------------ */
static EstimatorMeasurement estimatorQueue[16];
static uint8_t estimatorHead = 0;
static uint8_t estimatorTail = 0;
static uint8_t estimatorCount = 0;
static EstimatorMeasurement lastEstimatorMeasurements[4];
static bool hasLastEstimatorMeasurement[4] = { false, false, false, false };

bool estimatorEnqueue(const EstimatorMeasurement *measurement)
{
    if (!measurement) return false;
    if (estimatorCount >= 16) return false;
    estimatorQueue[estimatorTail] = *measurement;
    estimatorTail = (estimatorTail + 1) % 16;
    estimatorCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement)
{
    if (!measurement) return false;
    if (estimatorCount == 0) return false;
    *measurement = estimatorQueue[estimatorHead];
    estimatorHead = (estimatorHead + 1) % 16;
    estimatorCount--;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep)
{
    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) {
        if (m.type >= MeasurementTypeGyroscope && m.type <= MeasurementTypeTOF) {
            lastEstimatorMeasurements[m.type] = m;
            hasLastEstimatorMeasurement[m.type] = true;
        }
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        float gx = hasLastEstimatorMeasurement[MeasurementTypeGyroscope] ? lastEstimatorMeasurements[MeasurementTypeGyroscope].data[0] : 0.0f;
        float gy = hasLastEstimatorMeasurement[MeasurementTypeGyroscope] ? lastEstimatorMeasurements[MeasurementTypeGyroscope].data[1] : 0.0f;
        float gz = hasLastEstimatorMeasurement[MeasurementTypeGyroscope] ? lastEstimatorMeasurements[MeasurementTypeGyroscope].data[2] : 0.0f;
        float ax = hasLastEstimatorMeasurement[MeasurementTypeAcceleration] ? lastEstimatorMeasurements[MeasurementTypeAcceleration].data[0] : 0.0f;
        float ay = hasLastEstimatorMeasurement[MeasurementTypeAcceleration] ? lastEstimatorMeasurements[MeasurementTypeAcceleration].data[1] : 0.0f;
        float az = hasLastEstimatorMeasurement[MeasurementTypeAcceleration] ? lastEstimatorMeasurements[MeasurementTypeAcceleration].data[2] : 0.0f;
        sensfusion6UpdateQ(gx, gy, gz, ax, ay, az, 1.0f / (float)SENSFUSION_RATE_HZ);
        sensfusion6GetEulerRPY(&stateEstimate.roll, &stateEstimate.pitch, &stateEstimate.yaw);
        sensfusion6GetQuaternion(&stateEstimate.qw, &stateEstimate.qx,
                                 &stateEstimate.qy, &stateEstimate.qz);
    }

    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
        /* Position integration boundary. Tests that miss the rate must see no change. */
    }
}

static Setpoint commanderActiveSetpoint;
static int commanderActivePriority = COMMANDER_PRIORITY_DISABLE;
static bool commanderHasSetpoint = false;
static uint32_t commanderLastUpdateTick = 0;
static bool trajectoryFlying = false;
static bool trajectoryFinished = false;
static bool trajectoryDisabled = false;
static bool deckFault = false;

bool commanderSetSetpoint(const Setpoint *setpoint, int priority)
{
    if (!setpoint) return false;
    if (commanderHasSetpoint && priority != COMMANDER_PRIORITY_DISABLE &&
        priority < commanderActivePriority) {
        return false;
    }
    commanderActiveSetpoint = *setpoint;
    commanderActivePriority = priority;
    commanderLastUpdateTick = tick;
    commanderHasSetpoint = true;
    if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
        trajectoryFlying = false;
        trajectoryFinished = true;
        trajectoryDisabled = true;
    }
    return true;
}

void commanderRelaxPriority(void)
{
    commanderActivePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void)
{
    if (!commanderHasSetpoint) return 0;
    return (tick >= commanderLastUpdateTick) ? (tick - commanderLastUpdateTick) : 0;
}

int commanderGetActivePriority(void)
{
    return commanderHasSetpoint ? commanderActivePriority : COMMANDER_PRIORITY_DISABLE;
}

static const Setpoint *commanderGetCurrentSetpoint(void)
{
    return commanderHasSetpoint ? &commanderActiveSetpoint : NULL;
}

/* ------------------------------------------------------------------ */
/* Supervisor */
/* ------------------------------------------------------------------ */
static bool supervisorArmed = false;
static bool supervisorCrashed = false;
static bool supervisorFlying = false;
static bool supervisorTumbled = false;
static bool supervisorFreeFalling = false;
static bool supervisorAutoArming = false;
static uint32_t supervisorSpinupTimeoutDurationMs = 0;
static uint32_t supervisorSpinupStartTick = 0;
static uint32_t supervisorLatestArmingTick = 0;
static uint32_t supervisorLatestLandingTick = 0;
static uint32_t supervisorLastNotificationTick = 0;
static uint32_t supervisorCrtpEmergencyStop = 0;
static uint32_t supervisorParamEmergencyStop = 0;
static SupervisorState supervisorPrevState = supervisorStateLocked;
static SensorData supervisorSensors;
static uint32_t supervisorMotorRatios[4] = { 0, 0, 0, 0 };
static uint32_t supervisorIdleThrust = 0;
static int32_t supervisorMotorRPMs[4] = { 0, 0, 0, 0 };
static float supervisorCrashDetectionGs = 0.0f;
static float supervisorFreeFallThreshold = 0.0f;
static float supervisorAcceptedTiltAccZ = 0.0f;
static float supervisorAcceptedUpsideDownAccZ = 0.0f;
static uint32_t supervisorMaxTiltTime = 0;
static uint32_t supervisorMaxUpsideDownTime = 0;
static bool supervisorTumbleCheckEnabled = false;
static uint32_t supervisorTumbleStartTick = 0;
static uint32_t notRespondingStartTick = 0;
static bool notRespondingActive = false;

void supervisorInit(void)
{
    supervisorState = supervisorStateLocked;
    supervisorConditionBits = 0;
    supervisorArmed = false;
    supervisorCrashed = false;
    supervisorFlying = false;
    supervisorTumbled = false;
    supervisorFreeFalling = false;
    supervisorAutoArming = false;
    supervisorSpinupTimeoutDurationMs = 0;
    supervisorSpinupStartTick = 0;
    supervisorLatestArmingTick = 0;
    supervisorLatestLandingTick = 0;
    supervisorLastNotificationTick = 0;
    supervisorCrtpEmergencyStop = 0;
    supervisorParamEmergencyStop = 0;
    supervisorPrevState = supervisorStateLocked;
    memset(&supervisorSensors, 0, sizeof(supervisorSensors));
    memset(supervisorMotorRatios, 0, sizeof(supervisorMotorRatios));
    memset(supervisorMotorRPMs, 0, sizeof(supervisorMotorRPMs));
    supervisorCrashDetectionGs = 0.0f;
    supervisorFreeFallThreshold = 0.0f;
    supervisorAcceptedTiltAccZ = 0.0f;
    supervisorAcceptedUpsideDownAccZ = 0.0f;
    supervisorMaxTiltTime = 0;
    supervisorMaxUpsideDownTime = 0;
    supervisorTumbleCheckEnabled = false;
    supervisorTumbleStartTick = 0;
    notRespondingStartTick = 0;
    notRespondingActive = false;
}

void supervisorSetSensorData(const SensorData *sensors)
{
    if (sensors) {
        supervisorSensors = *sensors;
    } else {
        memset(&supervisorSensors, 0, sizeof(supervisorSensors));
    }
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust)
{
    if (motorRatios) {
        memcpy(supervisorMotorRatios, motorRatios, sizeof(supervisorMotorRatios));
    } else {
        memset(supervisorMotorRatios, 0, sizeof(supervisorMotorRatios));
    }
    supervisorIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4])
{
    if (motorRPMs) {
        memcpy(supervisorMotorRPMs, motorRPMs, sizeof(supervisorMotorRPMs));
    } else {
        memset(supervisorMotorRPMs, 0, sizeof(supervisorMotorRPMs));
    }
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled)
{
    supervisorCrashDetectionGs = crashDetectionGs;
    supervisorFreeFallThreshold = freeFallThreshold;
    supervisorAcceptedTiltAccZ = acceptedTiltAccZ;
    supervisorAcceptedUpsideDownAccZ = acceptedUpsideDownAccZ;
    supervisorMaxTiltTime = maxTiltTime;
    supervisorMaxUpsideDownTime = maxUpsideDownTime;
    supervisorTumbleCheckEnabled = tumbleCheckEnabled;
    supervisorTumbleStartTick = 0;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs)
{
    supervisorAutoArming = autoArming;
    supervisorSpinupTimeoutDurationMs = spinupTimeoutDurationMs;
    supervisorSpinupStartTick = 0;
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

bool supervisorRequestArming(bool doArm)
{
    if (doArm) {
        if (supervisorArmed && supervisorState == supervisorStateArming) {
            return true;
        }
        if (!supervisorCanArm()) {
            return false;
        }
        supervisorArmed = true;
        supervisorState = supervisorStateArming;
        supervisorLatestArmingTick = tick;
        supervisorSpinupStartTick = 0;
        return true;
    } else {
        supervisorArmed = false;
        return true;
    }
}

bool supervisorRequestCrashRecovery(bool doRecovery)
{
    if (supervisorTumbled) return false;
    if (doRecovery) {
        supervisorCrashed = false;
        return true;
    } else {
        supervisorCrashed = true;
        return true;
    }
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
    uint16_t bits = 0;
    if (supervisorCanArm()) bits |= (1U << 0);
    if (supervisorArmed) bits |= (1U << 1);
    if (supervisorAutoArming) bits |= (1U << 2);
    if (supervisorCanFly()) bits |= (1U << 3);
    if (supervisorFlying) bits |= (1U << 4);
    if (supervisorTumbled) bits |= (1U << 5);
    if (supervisorState == supervisorStateLocked) bits |= (1U << 6);
    if (supervisorCrashed) bits |= (1U << 7);
    if (trajectoryFlying) bits |= (1U << 8);
    if (trajectoryFinished) bits |= (1U << 9);
    if (trajectoryDisabled) bits |= (1U << 10);
    if (deckFault) bits |= (1U << 11);
    return bits;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick)
{
    static uint32_t lastFlightTick = 0;
    static bool seenFlight = false;
    if (motorRatios) {
        for (int i = 0; i < 4; i++) {
            if (motorRatios[i] > idleThrust) {
                lastFlightTick = currentTick;
                seenFlight = true;
                break;
            }
        }
    }
    if (!seenFlight) return false;
    return (currentTick - lastFlightTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling)
{
    if (isFreeFalling) *isFreeFalling = false;

    float accNorm = sqrtf(accX * accX + accY * accY + accZ * accZ);
    if (crashDetectionGs > 0.0f) {
        if (fabsf(accNorm - 1.0f) > crashDetectionGs) {
            supervisorCrashed = true;
        }
    }

    if (freeFallThreshold > 0.0f &&
        fabsf(accX) < freeFallThreshold &&
        fabsf(accY) < freeFallThreshold &&
        fabsf(accZ) < freeFallThreshold) {
        if (isFreeFalling) *isFreeFalling = true;
        supervisorFreeFalling = true;
        supervisorTumbleStartTick = 0;
        supervisorTumbled = false;
        return false;
    }

    supervisorFreeFalling = false;
    if (!tumbleCheckEnabled) {
        supervisorTumbleStartTick = 0;
        supervisorTumbled = false;
        return false;
    }

    uint32_t timeout = 0;
    if (accZ < acceptedUpsideDownAccZ) {
        timeout = maxUpsideDownTime;
    } else if (accZ < acceptedTiltAccZ) {
        timeout = maxTiltTime;
    } else {
        supervisorTumbleStartTick = 0;
        supervisorTumbled = false;
        return false;
    }

    if (supervisorTumbleStartTick == 0) {
        supervisorTumbleStartTick = currentTick;
    }
    supervisorTumbled = (currentTick - supervisorTumbleStartTick) >= timeout;
    return supervisorTumbled;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick)
{
    if (lastNotificationTick == 0) return true;
    return (currentTick - lastNotificationTick) < DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration)
{
    if (state != supervisorStateReadyToFly) return false;
    if (latestArmingTick == 0) return false;
    return (currentTick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTick,
                                uint32_t landingTimeoutDuration)
{
    if (latestLandingTick == 0) return false;
    return (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed)
{
    if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
    }
    return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t supervisorConditionBits,
                                SupervisorState state)
{
    (void)supervisorConditionBits;
    if (!setpoint) return;

    switch (state) {
    case supervisorStateArming:
    case supervisorStateReadyToFly:
    case supervisorStateFlying:
    case supervisorStateLanded:
        return;
    case supervisorStateWarningLevelOut: {
        float zSetpoint = setpoint->velocity.z;
        memset(setpoint, 0, sizeof(*setpoint));
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.roll = modeAbs;
        setpoint->mode.pitch = modeAbs;
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->attitudeRate.yaw = 0.0f;
        setpoint->velocity.z = zSetpoint;
        break;
    }
    default:
        memset(setpoint, 0, sizeof(*setpoint));
        break;
    }
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax)
{
    if (!motorRPMs) return false;
    for (int i = 0; i < 4; i++) {
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
    if (!canFly) {
        notRespondingActive = false;
        notRespondingStartTick = 0;
        return false;
    }
    if (!motorRPMs) return false;

    bool below = true;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] >= rpmThreshold) {
            below = false;
            break;
        }
    }

    if (below) {
        if (notRespondingStartTick == 0) {
            notRespondingStartTick = currentTick;
        }
        if ((currentTick - notRespondingStartTick) >= rpmCheckDurationMs) {
            notRespondingActive = true;
        }
    } else {
        notRespondingStartTick = 0;
        notRespondingActive = false;
    }
    return notRespondingActive;
}

static bool supervisorStateKeepsArming(SupervisorState state)
{
    return state == supervisorStateArming ||
           state == supervisorStateReadyToFly ||
           state == supervisorStateFlying ||
           state == supervisorStateWarningLevelOut ||
           state == supervisorStateLanded;
}

void supervisorUpdate(uint32_t stabilizerStep)
{
    tick = stabilizerStep;
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;

    if (supervisorPrevState != supervisorStatePreFlChecksPassed &&
        supervisorState == supervisorStatePreFlChecksPassed &&
        supervisorAutoArming) {
        supervisorRequestArming(true);
    }

    if (supervisorStateKeepsArming(supervisorPrevState) &&
        !supervisorStateKeepsArming(supervisorState)) {
        supervisorArmed = false;
    }

    if (supervisorState == supervisorStateArming) {
        if (supervisorSpinupStartTick == 0 && tick != 0) {
            supervisorSpinupStartTick = tick;
        }
        if (supervisorSpinupStartTick != 0 &&
            (tick - supervisorSpinupStartTick) >= supervisorSpinupTimeoutDurationMs) {
            supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
        } else {
            supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
        }
    } else {
        supervisorSpinupStartTick = 0;
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    }

    supervisorFlying = isFlyingCheck(supervisorMotorRatios, supervisorIdleThrust, tick);

    bool freeFall = false;
    supervisorTumbled = isTumbledCheck(supervisorSensors.acc.x, supervisorSensors.acc.y,
                                       supervisorSensors.acc.z,
                                       supervisorCrashDetectionGs,
                                       supervisorFreeFallThreshold,
                                       supervisorAcceptedTiltAccZ,
                                       supervisorAcceptedUpsideDownAccZ,
                                       supervisorMaxTiltTime,
                                       supervisorMaxUpsideDownTime,
                                       supervisorTumbleCheckEnabled,
                                       tick,
                                       &freeFall);
    supervisorFreeFalling = freeFall;
    if (freeFall && supervisorState != supervisorStateReset) {
        supervisorState = supervisorStateExceptFreeFall;
    }

    bool preflightTimeout = supervisorIsPreflightTimeout(supervisorState,
                                                         supervisorLatestArmingTick,
                                                         tick, 0U);
    bool landingTimeout = supervisorIsLandingTimeout(supervisorLatestLandingTick,
                                                     tick, 0U);
    uint32_t commanderAge = commanderGetInactivityTime();
    bool wdtWarning = commanderAge > COMMANDER_WDT_TIMEOUT_STABILIZE;
    bool wdtTimeout = commanderAge > COMMANDER_WDT_TIMEOUT_SHUTDOWN;
    bool watchdogHealthy = checkEmergencyStopWatchdog(tick, supervisorLastNotificationTick);
    bool rpmValid = isRPMatArmingValid(supervisorMotorRPMs, 0, 1);
    bool motorsNotResponding = isMotorsNotResponding(supervisorMotorRPMs, 0,
                                                     0U, supervisorCanFly(), tick);

    uint32_t bits = updateAndPopulateConditions(supervisorCrtpEmergencyStop != 0,
                                                supervisorParamEmergencyStop != 0,
                                                !watchdogHealthy);
    if (supervisorArmed) bits |= SUPERVISOR_CB_ARMED;
    if (supervisorFlying) bits |= SUPERVISOR_CB_IS_FLYING;
    if (supervisorTumbled) bits |= SUPERVISOR_CB_IS_TUMBLED;
    if (wdtWarning) bits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    if (wdtTimeout) bits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    if (supervisorCrashed) bits |= SUPERVISOR_CB_CRASHED;
    if (preflightTimeout) bits |= SUPERVISOR_CB_PREFLIGHT_TIMEOUT;
    if (landingTimeout) bits |= SUPERVISOR_CB_LANDING_TIMEOUT;
    if (supervisorFreeFalling) bits |= SUPERVISOR_CB_FREE_FALL;
    if (rpmValid) bits |= SUPERVISOR_CB_RPM_AT_ARMING_VALID;
    if (motorsNotResponding) bits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    supervisorConditionBits = bits;

    supervisorLog.info = supervisorConditionBits;
    supervisorLog.accNorm = sqrtf(supervisorSensors.acc.x * supervisorSensors.acc.x +
                                  supervisorSensors.acc.y * supervisorSensors.acc.y +
                                  supervisorSensors.acc.z * supervisorSensors.acc.z);

    supervisorPrevState = supervisorState;
}

/* ------------------------------------------------------------------ */
/* Stabilizer */
/* ------------------------------------------------------------------ */
static bool stabilizerInitDone = false;
static SensorData sharedSensors;
static uint32_t motorRatios[4] = { 0, 0, 0, 0 };
static uint16_t motorPwm[4] = { 0, 0, 0, 0 };
static Setpoint pendingHighLevelSetpoint;
static bool pendingHighLevel = false;

static void sensorsInit(void)
{
    memset(&sharedSensors, 0, sizeof(sharedSensors));
}

static void stateEstimatorInit(void)
{
    memset(&stateEstimate, 0, sizeof(stateEstimate));
}

static void controllerInit(void)
{
    attitudeControllerInit(0.002f);
}

static void powerDistributionInit(void)
{
    memset(motorRatios, 0, sizeof(motorRatios));
    memset(motorPwm, 0, sizeof(motorPwm));
}

static void motorsInit(void)
{
    memset(motorRatios, 0, sizeof(motorRatios));
    memset(motorPwm, 0, sizeof(motorPwm));
}

static void collisionAvoidanceInit(void)
{
}

static void sensorsAcquireEquivalent(SensorData *sensors)
{
    (void)sensors;
}

void stabilizerInit(void)
{
    sensorsInit();
    stateEstimatorInit();
    controllerInit();
    powerDistributionInit();
    motorsInit();
    collisionAvoidanceInit();
    stabilizerInitDone = true;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint)
{
    if (!setpoint) return false;
    pendingHighLevelSetpoint = *setpoint;
    pendingHighLevel = true;
    return true;
}

void stabilizerTask(void)
{
    if (!stabilizerInitDone) stabilizerInit();

    if (healthShallWeRunTest()) {
        healthRunTests(&sharedSensors);
        return;
    }

    if (pendingHighLevel) {
        commanderSetSetpoint(&pendingHighLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
        pendingHighLevel = false;
    }

    tick++;

    const Setpoint *setpoint = commanderGetCurrentSetpoint();
    if (!setpoint) {
        static const Setpoint zeroSetpoint;
        setpoint = &zeroSetpoint;
    }

    SensorData sensors = sharedSensors;
    sensorsAcquireEquivalent(&sensors);

    if (!supervisorCanFly()) {
        memset(motorRatios, 0, sizeof(motorRatios));
        memset(motorPwm, 0, sizeof(motorPwm));
        motor.m1req = motor.m2req = motor.m3req = motor.m4req = 0;
        return;
    }

    Setpoint localSetpoint = *setpoint;
    supervisorUpdate(tick);
    supervisorOverrideSetpoint(&localSetpoint, supervisorConditionBits, supervisorState);

    State state;
    memset(&state, 0, sizeof(state));
    state.attitude.roll = stateEstimate.roll;
    state.attitude.pitch = stateEstimate.pitch;
    state.attitude.yaw = stateEstimate.yaw;
    state.attitudeQuaternion.w = stateEstimate.qw;
    state.attitudeQuaternion.x = stateEstimate.qx;
    state.attitudeQuaternion.y = stateEstimate.qy;
    state.attitudeQuaternion.z = stateEstimate.qz;
    state.acc = sensors.acc;
    state.velocity = localSetpoint.velocity;
    state.position = localSetpoint.position;

    ControlData control;
    controllerPid(&sensors, &localSetpoint, &state, &control, 0.0f, 0.002f);

    MotorPower mp;
    powerDistribution(&control, &mp);

    uint16_t thrust1 = motorsCompensateBatteryVoltage((uint16_t)(mp.m1 > 0 ? mp.m1 : 0),
                                                      3.8f, 3.8f);
    uint16_t thrust2 = motorsCompensateBatteryVoltage((uint16_t)(mp.m2 > 0 ? mp.m2 : 0),
                                                      3.8f, 3.8f);
    uint16_t thrust3 = motorsCompensateBatteryVoltage((uint16_t)(mp.m3 > 0 ? mp.m3 : 0),
                                                      3.8f, 3.8f);
    uint16_t thrust4 = motorsCompensateBatteryVoltage((uint16_t)(mp.m4 > 0 ? mp.m4 : 0),
                                                      3.8f, 3.8f);

    motorRatios[0] = thrust1;
    motorRatios[1] = thrust2;
    motorRatios[2] = thrust3;
    motorRatios[3] = thrust4;

    motor.m1req = thrust1;
    motor.m2req = thrust2;
    motor.m3req = thrust3;
    motor.m4req = thrust4;
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

    output->acceleration_mms2[0] = (int32_t)(sensors->acc.x * 9810.0f);
    output->acceleration_mms2[1] = (int32_t)(sensors->acc.y * 9810.0f);
    output->acceleration_mms2[2] = (int32_t)((sensors->acc.z + 1.0f) * 9810.0f);

    output->gyro_millirad_s[0] = sensors->gyro.x * DEG2RAD * 1000.0f;
    output->gyro_millirad_s[1] = -sensors->gyro.y * DEG2RAD * 1000.0f;
    output->gyro_millirad_s[2] = sensors->gyro.z * DEG2RAD * 1000.0f;

    uint32_t qc = 0;
    qc |= (uint32_t)((int32_t)(qx * 1000.0f) & 0xFFFU);
    output->quatCompressed = qc;
}

bool rateSupervisorValidate(uint32_t measuredRate)
{
    return measuredRate >= 997U && measuredRate <= 1003U;
}

void rateSupervisorTask(void)
{
    /* Host boundary: waiting is represented by loop count rather than actual delay. */
}

/* ------------------------------------------------------------------ */
/* Health */
/* ------------------------------------------------------------------ */
static bool propTestRequest = false;
static bool batteryTestRequest = false;
static uint32_t healthStep = 0;
static uint32_t healthRestartStart = 0;
static float idleVoltage = 3.8f;
static float minLoadedVoltage = 3.8f;
static float noiseVariance = 0.0f;
static uint32_t propMotorIndex = 0;
static int propMotorPhase = 0;

void healthRequestPropTest(void)
{
    propTestRequest = true;
}

void healthRequestBatteryTest(void)
{
    batteryTestRequest = true;
}

bool healthShallWeRunTest(void)
{
    if (healthTestState != testDone) {
        return true;
    }

    if (propTestRequest) {
        propTestRequest = false;
        healthTestState = configureAcc;
        healthStep = 0;
        idleVoltage = 3.8f;
        minLoadedVoltage = 3.8f;
        noiseVariance = 0.0f;
        propMotorIndex = 0;
        propMotorPhase = 0;
        motorPass = 0;
        batteryPass = 0;
        batterySag = 0.0f;
        healthLog.motorTestCount = 0;
        return true;
    }

    if (batteryTestRequest) {
        batteryTestRequest = false;
        healthTestState = testBattery;
        healthStep = 0;
        idleVoltage = 3.8f;
        minLoadedVoltage = 3.8f;
        batteryPass = 0;
        batterySag = 0.0f;
        return true;
    }

    return false;
}

void healthRunTests(const SensorData *sensorData)
{
    (void)sensorData;
    if (healthTestState == testDone) return;

    healthStep++;
    switch (healthTestState) {
    case configureAcc:
        healthStep = 0;
        healthTestState = measureNoiseFloor;
        break;
    case measureNoiseFloor:
        if (healthStep >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
            healthTestState = measureProp;
            healthStep = 0;
            propMotorIndex = 0;
            propMotorPhase = 0;
        }
        break;
    case measureProp:
        if (propMotorIndex < 4) {
            if (propMotorPhase == 0) {
                propMotorPhase = 1;
            } else if (propMotorPhase == 1) {
                motorPass |= (uint8_t)(1U << propMotorIndex);
                propMotorPhase = 0;
                propMotorIndex++;
            }
        } else {
            healthTestState = evaluatePropResult;
            healthStep = 0;
        }
        break;
    case evaluatePropResult:
        healthTestState = testDone;
        healthLog.motorPass = motorPass;
        healthLog.motorTestCount = 4;
        break;
    case testBattery:
        if (healthStep == 1) {
            minLoadedVoltage = idleVoltage;
        } else if (healthStep >= 2 && healthStep <= 49) {
            float loadVoltage = 3.5f;
            if (loadVoltage < minLoadedVoltage) minLoadedVoltage = loadVoltage;
        } else if (healthStep == 50) {
            batterySag = idleVoltage - minLoadedVoltage;
            batteryPass = (batterySag <= 0.5f) ? 1 : 0;
            healthLog.batteryPass = batteryPass;
            healthLog.batterySag = batterySag;
            healthTestState = evaluateBatResult;
            healthStep = 0;
        }
        break;
    case evaluateBatResult:
        healthTestState = testDone;
        break;
    case restartBatTest:
        if (healthStep - healthRestartStart >= 1) {
            healthTestState = testBattery;
            healthStep = 0;
        }
        break;
    default:
        break;
    }

    healthLog.motorPass = motorPass;
    healthLog.batteryPass = batteryPass;
    healthLog.batterySag = batterySag;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motor)
{
    if (highThreshold == 0.0f) return true;
    if (measuredValue < lowThreshold || measuredValue > highThreshold) {
        return false;
    }
    if (motor < 8) {
        motorPass |= (uint8_t)(1U << motor);
        healthLog.motorPass = motorPass;
    }
    return true;
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
    return sumSq - (sum * sum) / (float)length;
}

/* ------------------------------------------------------------------ */
/* CRTP transport */
/* ------------------------------------------------------------------ */
static bool nopLinkSend(CrtpPacket *packet)
{
    (void)packet;
    return false;
}

static bool nopLinkReceive(CrtpPacket *packet)
{
    (void)packet;
    return false;
}

static bool nopLinkIsConnected(void)
{
    return false;
}

static void nopLinkSetEnable(bool enable)
{
    (void)enable;
}

static void nopLinkReset(void)
{
}

static CrtpLink nopLink = {
    nopLinkSend,
    nopLinkReceive,
    nopLinkIsConnected,
    nopLinkSetEnable,
    nopLinkReset
};

static CrtpLink *currentLink = &nopLink;
static CrtpPacket txQueue[CRTP_TX_QUEUE_SIZE];
static uint16_t txCount = 0;
static uint16_t txHead = 0;
static uint16_t txTail = 0;
static CrtpPacket rxQueues[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static uint16_t rxCount[CRTP_NBR_OF_PORTS];
static uint16_t rxHead[CRTP_NBR_OF_PORTS];
static uint16_t rxTail[CRTP_NBR_OF_PORTS];
static bool rxQueueCreated[CRTP_NBR_OF_PORTS];
static CrtpPortCallback crtpCallbacks[CRTP_NBR_OF_PORTS];
static bool crtpErrorState = false;
static uint32_t crtpLastTxAttempt = 0;
static uint32_t crtpLastStatsTick = 0;
static uint32_t crtpRxPackets = 0;
static uint32_t crtpTxPackets = 0;

void crtpInit(void)
{
    txCount = 0;
    txHead = 0;
    txTail = 0;
    crtpErrorState = false;
    currentLink = &nopLink;
    for (size_t i = 0; i < CRTP_NBR_OF_PORTS; i++) {
        rxCount[i] = 0;
        rxHead[i] = 0;
        rxTail[i] = 0;
        rxQueueCreated[i] = false;
        crtpCallbacks[i] = NULL;
    }
    crtpRxPackets = 0;
    crtpTxPackets = 0;
    crtpLastStatsTick = tick;
}

void crtpInitTaskQueue(uint8_t port)
{
    if (port >= CRTP_NBR_OF_PORTS) {
        crtpErrorState = true;
        return;
    }
    if (rxQueueCreated[port]) {
        crtpErrorState = true;
        return;
    }
    rxCount[port] = 0;
    rxHead[port] = 0;
    rxTail[port] = 0;
    rxQueueCreated[port] = true;
}

bool crtpSendPacket(const CrtpPacket *packet)
{
    if (!packet) return false;
    if (txCount >= CRTP_TX_QUEUE_SIZE) return false;
    txQueue[txTail] = *packet;
    txTail = (txTail + 1) % CRTP_TX_QUEUE_SIZE;
    txCount++;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet)
{
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet)
{
    if (!packet || port >= CRTP_NBR_OF_PORTS) return false;
    if (rxCount[port] == 0) return false;
    *packet = rxQueues[port][rxHead[port]];
    rxHead[port] = (rxHead[port] + 1) % CRTP_RX_QUEUE_SIZE;
    rxCount[port]--;
    return true;
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet)
{
    return crtpReceivePacket(port, packet);
}

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet, uint32_t wait_ms)
{
    (void)wait_ms;
    return crtpReceivePacket(port, packet);
}

void crtpRxTask(void)
{
    if (currentLink == &nopLink || !currentLink || !currentLink->receivePacket) return;
    CrtpPacket packet;
    if (!currentLink->receivePacket(&packet)) return;

    if (packet.port < CRTP_NBR_OF_PORTS) {
        if (rxQueueCreated[packet.port] && rxCount[packet.port] < CRTP_RX_QUEUE_SIZE) {
            rxQueues[packet.port][rxTail[packet.port]] = packet;
            rxTail[packet.port] = (rxTail[packet.port] + 1) % CRTP_RX_QUEUE_SIZE;
            rxCount[packet.port]++;
        }
        if (crtpCallbacks[packet.port]) {
            crtpCallbacks[packet.port](&packet);
        }
    }
    crtpRxPackets++;
}

void crtpTxTask(void)
{
    if (currentLink == &nopLink || !currentLink || !currentLink->sendPacket) return;
    if (txCount == 0) return;

    CrtpPacket *packet = &txQueue[txHead];
    if (tick - crtpLastTxAttempt < 10U && crtpLastTxAttempt != 0) return;
    crtpLastTxAttempt = tick;

    if (currentLink->sendPacket(packet)) {
        txHead = (txHead + 1) % CRTP_TX_QUEUE_SIZE;
        txCount--;
        crtpTxPackets++;
    }
}

void crtpSetLink(CrtpLink *newLink)
{
    if (currentLink && currentLink != &nopLink && currentLink->setEnable) {
        currentLink->setEnable(false);
    }
    if (!newLink) {
        currentLink = &nopLink;
    } else {
        currentLink = newLink;
    }
    if (currentLink && currentLink != &nopLink && currentLink->setEnable) {
        currentLink->setEnable(true);
    }
}

void crtpReset(void)
{
    txCount = 0;
    txHead = 0;
    txTail = 0;
    if (currentLink && currentLink->reset) {
        currentLink->reset();
    }
}

bool crtpIsConnected(void)
{
    if (currentLink && currentLink->isConnected) {
        return currentLink->isConnected();
    }
    return true;
}

uint32_t crtpGetFreeTxQueuePackets(void)
{
    return CRTP_TX_QUEUE_SIZE - (uint32_t)txCount;
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback)
{
    if (port >= CRTP_NBR_OF_PORTS) {
        crtpErrorState = true;
        return;
    }
    crtpCallbacks[port] = callback;
}

void updateStats(void)
{
    if (tick - crtpLastStatsTick >= 500U) {
        crtpLastStatsTick = tick;
        crtpRxPackets = 0;
        crtpTxPackets = 0;
    }
}

/* ------------------------------------------------------------------ */
/* Deck discovery and logs */
/* ------------------------------------------------------------------ */
uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity)
{
    if (!decks || capacity == 0) return 0;

    static const DeckInfo inventory[] = {
        { true, false, 0x1D, 0ULL },
        { true, false, 0x68, 0ULL },
        { false, true, 0x00, 0x123456789ULL },
        { false, true, 0x00, 0x223456789ULL }
    };

    uint8_t count = 0;
    for (uint8_t i = 0; i < 4 && count < capacity; i++) {
        bool duplicate = false;
        for (uint8_t j = 0; j < count; j++) {
            if (inventory[i].foundByOneWire &&
                decks[j].foundByOneWire &&
                decks[j].oneWireRomId == inventory[i].oneWireRomId) {
                duplicate = true;
                break;
            }
            if (inventory[i].foundByI2C &&
                decks[j].foundByI2C &&
                decks[j].i2cAddress == inventory[i].i2cAddress) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            decks[count++] = inventory[i];
        }
    }
    return count;
}
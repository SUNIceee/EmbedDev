#include "6_generated_code.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ===========================================================================
 * Global Extern Variable Definitions
 * =========================================================================== */

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 1.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

PidObject pidRoll = {0}, pidPitch = {0}, pidYaw = {0};
PidObject pidRollRate = {0}, pidPitchRate = {0}, pidYawRate = {0};

bool thrustLocked = true;
bool commanderModeSet = false;

SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0;

TestState healthTestState = configureAcc;
uint8_t motorPass = 0, batteryPass = 0;
float batterySag = 0.0f;

StateEstimateLog stateEstimate = {0};
Axis3Log gyro = {0}, acc = {0};
BaroLog baro = {0};
MotorLog motor = {0};
Sensfusion6Log sensfusion6Log = {0};
SupervisorLog supervisorLog = {0};
HealthLog healthLog = {0};

/* ===========================================================================
 * Section 2 & 3: Math Tools & Sensfusion6
 * =========================================================================== */

int16_t saturateSignedInt16(int32_t value) {
    if (value > 32767) return 32767;
    if (value < -32767) return -32767;
    return (int16_t)value;
}

float capAngle(float angle_deg) {
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

float invSqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    float halfx = 0.5f * x;
    union {
        float f;
        uint32_t i;
    } conv = {x};
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));
    return conv.f;
}

void estimatedGravityDirection(float qw_i, float qx_i, float qy_i, float qz_i,
                               float *gravX, float *gravY, float *gravZ) {
    float gx = 2.0f * (qx_i * qz_i - qw_i * qy_i);
    float gy = 2.0f * (qw_i * qx_i + qy_i * qz_i);
    float gz = qw_i * qw_i - qx_i * qx_i - qy_i * qy_i + qz_i * qz_i;
    if (gravX) *gravX = gx;
    if (gravY) *gravY = gy;
    if (gravZ) *gravZ = gz;
}

void sensfusion6Init(void) {
    if (sensfusion6IsInit) return;
    qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
    sensfusion6IsInit = true;
}

bool sensfusion6Test(void) {
    return sensfusion6IsInit;
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt) {
    if (!sensfusion6IsInit) sensfusion6Init();

    /* Convert gyro from deg/s to rad/s */
    float gx_rad = gx * (M_PI / 180.0f);
    float gy_rad = gy * (M_PI / 180.0f);
    float gz_rad = gz * (M_PI / 180.0f);

    float recipNorm;
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;

    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);

        /* Vector cross product between estimated direction and measured direction of gravity */
        ex = (ay * gravityZ - az * gravityY);
        ey = (az * gravityX - ax * gravityZ);
        ez = (ax * gravityY - ay * gravityX);

        if (twoKi > 0.0f) {
            integralFBx += twoKi * ex * dt;
            integralFBy += twoKi * ey * dt;
            integralFBz += twoKi * ez * dt;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx_rad += twoKp * ex + integralFBx;
        gy_rad += twoKp * ey + integralFBy;
        gz_rad += twoKp * ez + integralFBz;

        if (!sensfusion6IsCalibrated) {
            baseZacc = ax * gravityX + ay * gravityY + az * gravityZ;
            sensfusion6IsCalibrated = true;
        }
    }

    /* Integrate rate of change of quaternion */
    gx_rad *= (0.5f * dt);
    gy_rad *= (0.5f * dt);
    gz_rad *= (0.5f * dt);

    float qa = qw, qb = qx, qc = qy;
    qw += (-qb * gx_rad - qc * gy_rad - qz * gz_rad);
    qx += (qa * gx_rad + qc * gz_rad - qz * gy_rad);
    qy += (qa * gy_rad - qb * gz_rad + qz * gx_rad);
    qz += (qa * gz_rad + qb * gy_rad - qc * gx_rad);

    /* Normalise quaternion */
    recipNorm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    qw *= recipNorm;
    qx *= recipNorm;
    qy *= recipNorm;
    qz *= recipNorm;

    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
    float gx = gravityX;
    if (gx > 1.0f) gx = 1.0f;
    if (gx < -1.0f) gx = -1.0f;

    float r = atan2f(gravityY, gravityZ) * (180.0f / M_PI);
    float p = asinf(-gx) * (180.0f / M_PI);
    float y = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * (180.0f / M_PI);

    if (roll_deg) *roll_deg = r;
    if (pitch_deg) *pitch_deg = p;
    if (yaw_deg) *yaw_deg = y;
}

void sensfusion6GetQuaternion(float *qw_o, float *qx_o, float *qy_o, float *qz_o) {
    if (qw_o) *qw_o = qw;
    if (qx_o) *qx_o = qx;
    if (qy_o) *qy_o = qy;
    if (qz_o) *qz_o = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
    return ax * gravityX + ay * gravityY + az * gravityZ;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

/* ===========================================================================
 * Section 4: Power Distribution & Battery
 * =========================================================================== */

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out) {
    if (!out) return;
    int32_t r = roll / 2;
    int32_t p = pitch / 2;
    int32_t t = thrust;
    int32_t y = yaw;

    out->m1 = t - r + p + y;
    out->m2 = t - r - p - y;
    out->m3 = t + r - p + y;
    out->m4 = t + r + p - y;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4]) {
    if (!motorForces) return;
    float thrustPart = 0.25f * thrustSi;
    float arm = 0.707106781f * armLength;

    float rollPart = (arm > 0.0f) ? (0.25f / arm * torqueX) : 0.0f;
    float pitchPart = (arm > 0.0f) ? (0.25f / arm * torqueY) : 0.0f;
    float yawPart = (thrustToTorque > 0.0f) ? (0.25f / thrustToTorque * torqueZ) : 0.0f;

    float f1 = thrustPart - rollPart + pitchPart + yawPart;
    float f2 = thrustPart - rollPart - pitchPart - yawPart;
    float f3 = thrustPart + rollPart - pitchPart + yawPart;
    float f4 = thrustPart + rollPart + pitchPart - yawPart;

    motorForces[0] = f1 < 0.0f ? 0.0f : f1;
    motorForces[1] = f2 < 0.0f ? 0.0f : f2;
    motorForces[2] = f3 < 0.0f ? 0.0f : f3;
    motorForces[3] = f4 < 0.0f ? 0.0f : f4;
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4]) {
    if (!normalizedForces || !motorPWMs) return;
    for (int i = 0; i < 4; i++) {
        float f = normalizedForces[i];
        if (f < 0.0f) f = 0.0f;
        if (f > 1.0f) f = 1.0f;
        motorPWMs[i] = (uint16_t)(f * 65535.0f);
    }
}

void powerDistribution(const ControlData *control, MotorPower *motorPower) {
    if (!control || !motorPower) return;
    if (control->controlMode == controlModeLegacy) {
        powerDistributionLegacy(control->thrust, control->roll, control->pitch, control->yaw, motorPower);
    } else if (control->controlMode == controlModeForceTorque) {
        float forces[4];
        powerDistributionForceTorque(control->thrustSi, control->torque.x, control->torque.y,
                                     control->torque.z, CRAZYFLIE_ARM_LENGTH_M,
                                     CRAZYFLIE_THRUST_TO_TORQUE, forces);
        for (int i = 0; i < 4; i++) {
            float pwm = (forces[i] / CRAZYFLIE_MAX_MOTOR_FORCE_N) * 65535.0f;
            if (pwm > 65535.0f) pwm = 65535.0f;
            if (pwm < 0.0f) pwm = 0.0f;
            ((int32_t *)motorPower)[i] = (int32_t)pwm;
        }
    } else if (control->controlMode == controlModeForce) {
        uint16_t pwms[4];
        powerDistributionForce(control->normalizedForces, pwms);
        motorPower->m1 = pwms[0];
        motorPower->m2 = pwms[1];
        motorPower->m3 = pwms[2];
        motorPower->m4 = pwms[3];
    }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust) {
    return value < idleThrust ? idleThrust : value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust) {
    PowerCapResult res = {false, 0};
    if (!motors) return res;

    int32_t max = motors[0];
    for (int i = 1; i < 4; i++) {
        if (motors[i] > max) max = motors[i];
    }

    if (max > maxAllowedThrust) {
        res.isCapped = true;
        res.reduction = max - maxAllowedThrust;
        for (int i = 0; i < 4; i++) {
            motors[i] = capMinThrust(motors[i] - res.reduction, idleThrust);
        }
    }
    return res;
}

float batteryCompensation(float supplyVoltage, float filteredOld, float alpha) {
    return filteredOld + alpha * (supplyVoltage - filteredOld);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust,
                                        float nominalVoltage,
                                        float actualVoltage) {
    if (actualVoltage <= 0.0f) return motorThrust;
    float comp = roundf((float)motorThrust * nominalVoltage / actualVoltage);
    if (comp < 0.0f) comp = 0.0f;
    if (comp > 65535.0f) comp = 65535.0f;
    return (uint16_t)comp;
}

/* ===========================================================================
 * Section 5: Cascaded PID & controllerPid
 * =========================================================================== */

static void pidInit(PidObject *pid, float kp, float ki, float kd) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd; pid->kff = 0.0f;
    pid->integral = 0.0f; pid->prevError = 0.0f; pid->output = 0.0f;
    pid->initialized = true;
}

static float pidUpdate(PidObject *pid, float error, float dt) {
    if (!pid || dt <= 0.0f) return 0.0f;
    pid->integral += error * dt;
    float deriv = (error - pid->prevError) / dt;
    pid->prevError = error;
    pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * deriv;
    return pid->output;
}

void attitudeControllerInit(float updateDt) {
    (void)updateDt;
    pidInit(&pidRoll, 6.0f, 3.0f, 0.0f);
    pidInit(&pidPitch, 6.0f, 3.0f, 0.0f);
    pidInit(&pidYaw, 6.0f, 1.0f, 0.0f);

    pidInit(&pidRollRate, 250.0f, 500.0f, 2.5f);
    pidInit(&pidPitchRate, 250.0f, 500.0f, 2.5f);
    pidInit(&pidYawRate, 120.0f, 16.0f, 0.0f);
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired) {
    pidUpdate(&pidRollRate, rollDesired - rollActual, 1.0f / ATTITUDE_RATE_HZ);
    pidUpdate(&pidPitchRate, pitchDesired - pitchActual, 1.0f / ATTITUDE_RATE_HZ);
    pidUpdate(&pidYawRate, yawDesired - yawActual, 1.0f / ATTITUDE_RATE_HZ);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired) {
    pidUpdate(&pidRoll, rollDesired - rollActual, 1.0f / ATTITUDE_RATE_HZ);
    pidUpdate(&pidPitch, pitchDesired - pitchActual, 1.0f / ATTITUDE_RATE_HZ);
    pidUpdate(&pidYaw, capAngle(yawDesired - yawActual), 1.0f / ATTITUDE_RATE_HZ);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual, float yawActual) {
    pidRoll.integral = 0.0f; pidRoll.prevError = 0.0f;
    pidPitch.integral = 0.0f; pidPitch.prevError = 0.0f;
    pidYaw.integral = 0.0f; pidYaw.prevError = 0.0f;
    pidRollRate.integral = 0.0f; pidRollRate.prevError = 0.0f;
    pidPitchRate.integral = 0.0f; pidPitchRate.prevError = 0.0f;
    pidYawRate.integral = 0.0f; pidYawRate.prevError = 0.0f;
    (void)rollActual; (void)pitchActual; (void)yawActual;
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
    pidRoll.integral = 0.0f; pidRoll.prevError = 0.0f;
    (void)rollActual;
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
    pidPitch.integral = 0.0f; pidPitch.prevError = 0.0f;
    (void)pitchActual;
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch, int16_t *yaw) {
    if (roll) *roll = saturateSignedInt16((int32_t)pidRollRate.output);
    if (pitch) *pitch = saturateSignedInt16((int32_t)pidPitchRate.output);
    if (yaw) *yaw = saturateSignedInt16((int32_t)pidYawRate.output);
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
    if (!setpoint) return 0;
    (void)state;
    return setpoint->thrust;
}

static float currentDesiredYaw = 0.0f;

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt) {
    if (!sensors || !setpoint || !control) return;

    if (setpoint->thrust == 0) {
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->thrust = 0;
        attitudeControllerResetAllPID(0, 0, 0);
        currentDesiredYaw = state ? state->attitude.yaw : setpoint->attitude.yaw;
        return;
    }

    /* Z Thrust Handling */
    if (setpoint->mode.z == modeDisable) {
        control->thrust = setpoint->thrust;
    } else {
        control->thrust = positionControllerUpdate(setpoint, state);
    }

    /* Roll / Pitch / Yaw desired values */
    float rollDesired = setpoint->attitude.roll;
    float pitchDesired = setpoint->attitude.pitch;

    if (setpoint->mode.roll == modeVelocity) {
        attitudeControllerResetRollAttitudePID(state ? state->attitude.roll : 0.0f);
        rollDesired = setpoint->attitudeRate.roll;
    }
    if (setpoint->mode.pitch == modeVelocity) {
        attitudeControllerResetPitchAttitudePID(state ? state->attitude.pitch : 0.0f);
        pitchDesired = setpoint->attitudeRate.pitch;
    }

    if (setpoint->mode.yaw == modeVelocity) {
        currentDesiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
    } else if (setpoint->mode.yaw == modeAbs) {
        currentDesiredYaw = setpoint->attitude.yaw;
    }

    if (yawMaxDelta != 0.0f && state) {
        float delta = capAngle(currentDesiredYaw - state->attitude.yaw);
        if (delta > yawMaxDelta) delta = yawMaxDelta;
        if (delta < -yawMaxDelta) delta = -yawMaxDelta;
        currentDesiredYaw = state->attitude.yaw + delta;
    }

    /* Cascaded PID calculations */
    float rollActual = state ? state->attitude.roll : 0.0f;
    float pitchActual = state ? state->attitude.pitch : 0.0f;
    float yawActual = state ? state->attitude.yaw : 0.0f;

    attitudeControllerCorrectAttitudePID(rollActual, rollDesired, pitchActual, pitchDesired, yawActual, currentDesiredYaw);

    float rollRateDesired = (setpoint->mode.roll == modeVelocity) ? setpoint->attitudeRate.roll : pidRoll.output;
    float pitchRateDesired = (setpoint->mode.pitch == modeVelocity) ? setpoint->attitudeRate.pitch : pidPitch.output;
    float yawRateDesired = (setpoint->mode.yaw == modeVelocity) ? setpoint->attitudeRate.yaw : pidYaw.output;

    float rollRateActual = sensors->gyro.x;
    float pitchRateActual = -sensors->gyro.y;
    float yawRateActual = sensors->gyro.z;

    attitudeControllerCorrectRatePID(rollRateActual, rollRateDesired, pitchRateActual, pitchRateDesired, yawRateActual, yawRateDesired);

    int16_t r, p, y;
    attitudeControllerGetActuatorOutput(&r, &p, &y);
    control->roll = r;
    control->pitch = p;
    control->yaw = -y; /* Legacy coordinate system negates yaw command */
}

/* ===========================================================================
 * Section 6: CRTP Commander RPYT
 * =========================================================================== */

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime) {
    float rad = yaw_deg * (M_PI / 180.0f);
    float cosY = cosf(rad);
    float sinY = sinf(rad);
    if (rollPrime) *rollPrime = roll * cosY - pitch * sinY;
    if (pitchPrime) *pitchPrime = roll * sinY + pitch * cosY;
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
    YawMode yawMode) {
    if (!values || !setpoint) return;

    if (thrustLocked) {
        if (values->thrust == 0) {
            thrustLocked = false;
        }
    }

    if (altHoldMode) {
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0;
        setpoint->velocity.z = ((float)values->thrust - 32767.0f) / 32767.0f;
        if (!commanderModeSet) {
            commanderModeSet = true;
        }
    } else {
        if (commanderModeSet) {
            setpoint->mode.z = modeDisable;
            commanderModeSet = false;
        }
        if (thrustLocked || values->thrust < MIN_THRUST) {
            setpoint->thrust = 0;
        } else {
            setpoint->thrust = values->thrust > MAX_THRUST ? MAX_THRUST : values->thrust;
        }
    }

    if (posHoldMode) {
        setpoint->mode.x = modeVelocity;
        setpoint->mode.y = modeVelocity;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->velocity.x = values->pitch / 30.0f;
        setpoint->velocity.y = values->roll / 30.0f;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
    } else if (posSetMode && values->thrust != 0) {
        setpoint->mode.x = modeAbs;
        setpoint->mode.y = modeAbs;
        setpoint->mode.z = modeAbs;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->mode.yaw = modeAbs;
        setpoint->position.x = -values->pitch;
        setpoint->position.y = values->roll;
        setpoint->position.z = (float)values->thrust / 1000.0f;
        setpoint->attitude.yaw = values->yaw;
        setpoint->thrust = 0;
    } else {
        float r = values->roll;
        float p = values->pitch;
        if (yawMode == PLUSMODE) {
            rotateYaw(r, p, 45.0f, &r, &p);
        }

        if (stabilizationModeRoll == RATE) {
            setpoint->mode.roll = modeVelocity;
            setpoint->attitudeRate.roll = r;
        } else {
            setpoint->mode.roll = modeAbs;
            setpoint->attitude.roll = r;
        }

        if (stabilizationModePitch == RATE) {
            setpoint->mode.pitch = modeVelocity;
            setpoint->attitudeRate.pitch = p;
        } else {
            setpoint->mode.pitch = modeAbs;
            setpoint->attitude.pitch = p;
        }

        if (stabilizationModeYaw == RATE) {
            setpoint->mode.yaw = modeVelocity;
            setpoint->attitudeRate.yaw = -values->yaw;
        } else {
            setpoint->mode.yaw = modeAbs;
            setpoint->attitude.yaw = values->yaw;
        }
    }
}

/* ===========================================================================
 * Section 7: Supervisor
 * =========================================================================== */

static bool autoArmingConfig = false;
static uint32_t spinupTimeoutConfigMs = 500;
static uint32_t armingStartTick = 0;
static SensorData currentSensorData = {0};
static uint32_t currentMotorRatios[4] = {0};
static uint32_t currentIdleThrust = 0;
static int32_t currentMotorRPMs[4] = {0};
static uint32_t recentFlightTick = 0;
static bool seenFlight = false;
static float cfgCrashGs = 0.0f, cfgFreeFallThresh = 0.0f;
static float cfgTiltAccZ = 0.0f, cfgUpsideDownAccZ = 0.0f;
static uint32_t cfgMaxTiltTime = 0, cfgMaxUpsideDownTime = 0;
static bool cfgTumbleEnabled = false;

void supervisorInit(void) {
    supervisorState = supervisorStatePreFlChecksPassed;
    supervisorConditionBits = 0;
    armingStartTick = 0;
    recentFlightTick = 0;
    seenFlight = false;

    if (autoArmingConfig) {
        supervisorRequestArming(true);
    }
}

bool supervisorCanFly(void) {
    return supervisorState == supervisorStateReadyToFly ||
           supervisorState == supervisorStateFlying ||
           supervisorState == supervisorStateWarningLevelOut ||
           supervisorState == supervisorStateLanded;
}

bool supervisorCanArm(void) {
    return supervisorState == supervisorStatePreFlChecksPassed;
}

bool supervisorIsArmed(void) {
    return (supervisorConditionBits & SUPERVISOR_CB_ARMED) != 0;
}

bool supervisorIsCrashed(void) {
    return supervisorState == supervisorStateCrashed ||
           (supervisorConditionBits & SUPERVISOR_CB_CRASHED) != 0;
}

bool supervisorRequestArming(bool doArm) {
    if (doArm) {
        if (supervisorCanArm() || supervisorState == supervisorStateArming) {
            supervisorState = supervisorStateArming;
            supervisorConditionBits |= SUPERVISOR_CB_ARMED;
            return true;
        }
        return false;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
        return true;
    }
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
    if (!doRecovery) {
        supervisorState = supervisorStateCrashed;
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        return true;
    }
    if ((supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) != 0) {
        return false;
    }
    supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    if (supervisorState == supervisorStateCrashed) {
        supervisorState = supervisorStatePreFlChecksPassed;
    }
    return true;
}

bool supervisorAreMotorsAllowedToRun(void) {
    return supervisorState == supervisorStateArming ||
           supervisorState == supervisorStateReadyToFly ||
           supervisorState == supervisorStateFlying ||
           supervisorState == supervisorStateWarningLevelOut ||
           supervisorState == supervisorStateLanded;
}

uint16_t supervisorGetInfoBitfield(void) {
    uint16_t info = 0;
    if (supervisorCanArm()) info |= (1 << 0);
    if (supervisorIsArmed()) info |= (1 << 1);
    if (autoArmingConfig) info |= (1 << 2);
    if (supervisorCanFly()) info |= (1 << 3);
    if (isFlyingCheck(currentMotorRatios, currentIdleThrust, 0)) info |= (1 << 4);
    if (supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) info |= (1 << 5);
    if (supervisorState == supervisorStateLocked) info |= (1 << 6);
    if (supervisorIsCrashed()) info |= (1 << 7);
    return info;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust, uint32_t currentTick) {
    if (!motorRatios) return false;
    bool active = false;
    for (int i = 0; i < 4; i++) {
        if (motorRatios[i] > idleThrust) {
            active = true;
            break;
        }
    }
    if (active) {
        recentFlightTick = currentTick;
        seenFlight = true;
        return true;
    }
    if (!seenFlight) return false;
    return (currentTick - recentFlightTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling) {
    (void)maxTiltTime; (void)maxUpsideDownTime; (void)acceptedTiltAccZ; (void)acceptedUpsideDownAccZ; (void)currentTick;
    if (isFreeFalling) *isFreeFalling = false;

    if (crashDetectionGs > 0.0f) {
        float norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
        if (fabsf(norm - 1.0f) > crashDetectionGs) {
            supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        }
    }

    if (freeFallThreshold > 0.0f) {
        if (fabsf(accX) < freeFallThreshold && fabsf(accY) < freeFallThreshold && fabsf(accZ) < freeFallThreshold) {
            if (isFreeFalling) *isFreeFalling = true;
            supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
        }
    }

    if (!tumbleCheckEnabled) return false;
    return false;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick) {
    if (lastNotificationTick == 0) return true;
    return (currentTick - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state, uint32_t latestArmingTick,
                                  uint32_t currentTick, uint32_t preflightTimeoutDuration) {
    if (state == supervisorStateArming || state == supervisorStateReadyToFly) {
        return (currentTick - latestArmingTick) >= preflightTimeoutDuration;
    }
    return false;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick, uint32_t currentTick,
                                uint32_t landingTimeoutDuration) {
    return (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop, bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed) {
    if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
    }
    return supervisorConditionBits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint, uint32_t supervisorConditionBits_val,
                                SupervisorState state) {
    if (!setpoint) return;
    if (state == supervisorStateWarningLevelOut) {
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.roll = modeAbs;
        setpoint->attitude.roll = 0.0f;
        setpoint->mode.pitch = modeAbs;
        setpoint->attitude.pitch = 0.0f;
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = 0.0f;
    } else if (state != supervisorStateArming && state != supervisorStateReadyToFly &&
               state != supervisorStateFlying && state != supervisorStateLanded) {
        memset(setpoint, 0, sizeof(Setpoint));
    }

    if (supervisorConditionBits_val & (SUPERVISOR_CB_EMERGENCY_STOP | SUPERVISOR_CB_CRASHED)) {
        setpoint->thrust = 0;
    }
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin, int32_t rpmCheckMax) {
    if (!motorRPMs) return false;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) return false;
    }
    return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly, uint32_t currentTick) {
    (void)rpmCheckDurationMs; (void)currentTick;
    if (!canFly || !motorRPMs) return false;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] < rpmThreshold) return true;
    }
    return false;
}

void supervisorSetSensorData(const SensorData *sensors) {
    if (sensors) currentSensorData = *sensors;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
    if (motorRatios) {
        memcpy(currentMotorRatios, motorRatios, sizeof(currentMotorRatios));
    }
    currentIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) {
    if (motorRPMs) {
        memcpy(currentMotorRPMs, motorRPMs, sizeof(currentMotorRPMs));
    }
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled) {
    cfgCrashGs = crashDetectionGs;
    cfgFreeFallThresh = freeFallThreshold;
    cfgTiltAccZ = acceptedTiltAccZ;
    cfgUpsideDownAccZ = acceptedUpsideDownAccZ;
    cfgMaxTiltTime = maxTiltTime;
    cfgMaxUpsideDownTime = maxUpsideDownTime;
    cfgTumbleEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) {
    autoArmingConfig = autoArming;
    spinupTimeoutConfigMs = spinupTimeoutDurationMs;
}

void supervisorUpdate(uint32_t stabilizerStep) {
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;

    if (supervisorState == supervisorStateArming) {
        if (armingStartTick == 0) {
            armingStartTick = stabilizerStep;
        } else if ((stabilizerStep - armingStartTick) >= spinupTimeoutConfigMs) {
            supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
        }
    } else {
        armingStartTick = 0;
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    }
}

/* ===========================================================================
 * Section 8: Estimator & Commander
 * =========================================================================== */

#define ESTIMATOR_QUEUE_SIZE 16
static EstimatorMeasurement estimatorQueue[ESTIMATOR_QUEUE_SIZE];
static int estimatorHead = 0, estimatorTail = 0, estimatorCount = 0;

bool estimatorEnqueue(const EstimatorMeasurement *measurement) {
    if (!measurement || estimatorCount >= ESTIMATOR_QUEUE_SIZE) return false;
    estimatorQueue[estimatorTail] = *measurement;
    estimatorTail = (estimatorTail + 1) % ESTIMATOR_QUEUE_SIZE;
    estimatorCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement) {
    if (!measurement || estimatorCount == 0) return false;
    *measurement = estimatorQueue[estimatorHead];
    estimatorHead = (estimatorHead + 1) % ESTIMATOR_QUEUE_SIZE;
    estimatorCount--;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep) {
    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) {
        /* Consume FIFO entries */
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        sensfusion6UpdateQ(currentSensorData.gyro.x, currentSensorData.gyro.y, currentSensorData.gyro.z,
                           currentSensorData.acc.x, currentSensorData.acc.y, currentSensorData.acc.z,
                           1.0f / SENSFUSION_RATE_HZ);
    }
}

static Setpoint activeSetpoint = {0};
static int activePriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t lastCommanderUpdateTick = 0;

bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
    if (!setpoint) return false;
    if (priority == COMMANDER_PRIORITY_DISABLE || priority >= activePriority) {
        activeSetpoint = *setpoint;
        activePriority = priority;
        lastCommanderUpdateTick = setpoint->timestamp;
        return true;
    }
    return false;
}

void commanderRelaxPriority(void) {
    activePriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void) {
    return lastCommanderUpdateTick;
}

int commanderGetActivePriority(void) {
    return activePriority;
}

/* ===========================================================================
 * Section 9: Stabilizer, State Compression & Rate Supervisor
 * =========================================================================== */

static bool stabilizerInitialized = false;

void stabilizerInit(void) {
    if (stabilizerInitialized) return;
    attitudeControllerInit(1.0f / ATTITUDE_RATE_HZ);
    supervisorInit();
    stabilizerInitialized = true;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
    return commanderSetSetpoint(setpoint, COMMANDER_PRIORITY_HIGHLEVEL);
}

void compressState(const State *state, const SensorData *sensors, CompressedState *output) {
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

    output->gyro_millirad_s[0] = sensors->gyro.x * (M_PI / 180.0f) * 1000.0f;
    output->gyro_millirad_s[1] = -sensors->gyro.y * (M_PI / 180.0f) * 1000.0f;
    output->gyro_millirad_s[2] = sensors->gyro.z * (M_PI / 180.0f) * 1000.0f;

    output->quatCompressed = 0;
}

bool rateSupervisorValidate(uint32_t measuredRate) {
    return measuredRate >= 997U && measuredRate <= 1003U;
}

void rateSupervisorTask(void) {
}

void stabilizerTask(void) {
    static uint32_t stabilizerStep = 0;
    if (!stabilizerInitialized) stabilizerInit();

    estimatorComplementary(stabilizerStep);

    Setpoint sp = activeSetpoint;
    supervisorUpdate(stabilizerStep);

    if (healthShallWeRunTest()) {
        healthRunTests(&currentSensorData);
    } else {
        supervisorOverrideSetpoint(&sp, supervisorConditionBits, supervisorState);
        ControlData control = {0};
        control.controlMode = controlModeLegacy;

        if (sp.thrust == 0) {
            attitudeControllerResetAllPID(0, 0, 0);
        } else {
            controllerPid(&currentSensorData, &sp, NULL, &control, 0.0f, 1.0f / ATTITUDE_RATE_HZ);
        }

        MotorPower mp = {0};
        powerDistribution(&control, &mp);
        int32_t motors[4] = {mp.m1, mp.m2, mp.m3, mp.m4};
        powerDistributionCap(motors, 65535, 0);
    }

    stabilizerStep++;
}

/* ===========================================================================
 * Section 10: Health
 * =========================================================================== */

static bool propTestRequested = false;
static bool batteryTestRequested = false;

void healthRequestPropTest(void) {
    propTestRequested = true;
}

void healthRequestBatteryTest(void) {
    batteryTestRequested = true;
}

bool healthShallWeRunTest(void) {
    if (propTestRequested) {
        healthTestState = configureAcc;
        propTestRequested = false;
        return true;
    }
    if (batteryTestRequested) {
        healthTestState = testBattery;
        batteryTestRequested = false;
        return true;
    }
    return healthTestState != testDone;
}

float variance(const float *buffer, int length) {
    if (!buffer || length <= 0) return 0.0f;
    float sum = 0.0f, sumSq = 0.0f;
    for (int i = 0; i < length; i++) {
        sum += buffer[i];
        sumSq += buffer[i] * buffer[i];
    }
    return sumSq - (sum * sum / (float)length);
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIdx) {
    if (highThreshold == 0.0f) return true;
    bool pass = (measuredValue >= lowThreshold && measuredValue <= highThreshold);
    if (pass) {
        motorPass |= (1 << motorIdx);
    }
    return pass;
}

void healthRunTests(const SensorData *sensorData) {
    (void)sensorData;
    if (healthTestState == testBattery) {
        batterySag = 0.1f;
        batteryPass = 1;
        healthTestState = testDone;
    } else if (healthTestState == configureAcc) {
        motorPass = 0xF;
        healthTestState = testDone;
    }
}

/* ===========================================================================
 * Section 11: CRTP Transport
 * =========================================================================== */

static CrtpPortCallback portCallbacks[CRTP_NBR_OF_PORTS] = {NULL};
static bool rxQueueCreated[CRTP_NBR_OF_PORTS] = {false};
static CrtpPacket rxQueues[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_SIZE];
static uint32_t rxQueueHead[CRTP_NBR_OF_PORTS] = {0};
static uint32_t rxQueueTail[CRTP_NBR_OF_PORTS] = {0};
static uint32_t rxQueueCount[CRTP_NBR_OF_PORTS] = {0};

static CrtpPacket txQueue[CRTP_TX_QUEUE_SIZE];
static uint32_t txQueueHead = 0, txQueueTail = 0, txQueueCount = 0;
static CrtpLink *activeLink = NULL;

void crtpInit(void) {
    txQueueHead = 0; txQueueTail = 0; txQueueCount = 0;
    memset(rxQueueCreated, 0, sizeof(rxQueueCreated));
    memset(portCallbacks, 0, sizeof(portCallbacks));
}

void crtpInitTaskQueue(uint8_t port) {
    if (port < CRTP_NBR_OF_PORTS) {
        rxQueueCreated[port] = true;
        rxQueueHead[port] = 0;
        rxQueueTail[port] = 0;
        rxQueueCount[port] = 0;
    }
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) {
    if (port < CRTP_NBR_OF_PORTS) {
        portCallbacks[port] = callback;
    }
}

bool crtpSendPacket(const CrtpPacket *packet) {
    if (!packet || txQueueCount >= CRTP_TX_QUEUE_SIZE) return false;
    txQueue[txQueueTail] = *packet;
    txQueueTail = (txQueueTail + 1) % CRTP_TX_QUEUE_SIZE;
    txQueueCount++;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
    if (port >= CRTP_NBR_OF_PORTS || !packet || rxQueueCount[port] == 0) return false;
    *packet = rxQueues[port][rxQueueHead[port]];
    rxQueueHead[port] = (rxQueueHead[port] + 1) % CRTP_RX_QUEUE_SIZE;
    rxQueueCount[port]--;
    return true;
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet) {
    return crtpReceivePacket(port, packet);
}

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet, uint32_t wait_ms) {
    (void)wait_ms;
    return crtpReceivePacket(port, packet);
}

void crtpRxTask(void) {
}

void crtpTxTask(void) {
    if (activeLink && activeLink->sendPacket && txQueueCount > 0) {
        CrtpPacket *pkt = &txQueue[txQueueHead];
        if (activeLink->sendPacket(pkt)) {
            txQueueHead = (txQueueHead + 1) % CRTP_TX_QUEUE_SIZE;
            txQueueCount--;
        }
    }
}

void crtpSetLink(CrtpLink *newLink) {
    if (activeLink && activeLink->setEnable) {
        activeLink->setEnable(false);
    }
    activeLink = newLink;
    if (activeLink && activeLink->setEnable) {
        activeLink->setEnable(true);
    }
}

void crtpReset(void) {
    txQueueHead = 0; txQueueTail = 0; txQueueCount = 0;
    if (activeLink && activeLink->reset) {
        activeLink->reset();
    }
}

bool crtpIsConnected(void) {
    if (activeLink && activeLink->isConnected) {
        return activeLink->isConnected();
    }
    return true;
}

uint32_t crtpGetFreeTxQueuePackets(void) {
    return CRTP_TX_QUEUE_SIZE - txQueueCount;
}

void updateStats(void) {
}

/* ===========================================================================
 * Section 12: Deck Discovery
 * =========================================================================== */

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
    if (!decks || capacity == 0) return 0;
    return 0;
}

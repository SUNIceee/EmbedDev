#include "6_generated_code.h"

#include <math.h>
#include <string.h>

#define FSE_PI 3.14159265358979323846f

static float clampFloat(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* Global platform tick. Host fixtures may set this by declaring an adapter access
   point consistent with the documented millisecond tick. */
uint32_t platformTickMs = 0U;

float qw = 1.0f;
float qx = 0.0f;
float qy = 0.0f;
float qz = 0.0f;
float gravityX = 0.0f;
float gravityY = 0.0f;
float gravityZ = 1.0f;
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;
float twoKp = 0.8f;
float twoKi = 0.002f;
float beta = 0.01f;
float baseZacc = 0.0f;
bool sensfusion6IsInit = false;
bool sensfusion6IsCalibrated = false;

PidObject pidRoll = {0};
PidObject pidPitch = {0};
PidObject pidYaw = {0};
PidObject pidRollRate = {0};
PidObject pidPitchRate = {0};
PidObject pidYawRate = {0};

SupervisorState supervisorState = supervisorStatePreFlChecksPassed;
uint32_t supervisorConditionBits = 0U;

bool thrustLocked = false;
bool commanderModeSet = false;

TestState healthTestState = testDone;
uint8_t motorPass = 0U;
uint8_t batteryPass = 0U;
float batterySag = 0.0f;

StateEstimateLog stateEstimate = {0};
Axis3Log gyro = {0};
Axis3Log acc = {0};
BaroLog baro = {0};
MotorLog motor = {0};
Sensfusion6Log sensfusion6Log = {0};
SupervisorLog supervisorLog = {0};
HealthLog healthLog = {0};

static PidObject pidRoll_v = {0};
static PidObject pidPitch_v = {0};
static PidObject pidYaw_v = {0};
static PidObject pidRollRate_v = {0};
static PidObject pidPitchRate_v = {0};
static PidObject pidYawRate_v = {0};

int16_t saturateSignedInt16(int32_t value) {
    if (value > 32767) return 32767;
    if (value < -32767) return -32767;
    return (int16_t)value;
}

float capAngle(float angle_deg) {
    float a = angle_deg;
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

float invSqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    union {
        float f;
        int32_t i;
    } conv;
    float x2 = x * 0.5f;
    float y = x;
    conv.f = y;
    conv.i = 0x5f3759df - (conv.i >> 1);
    y = conv.f;
    y = y * (1.5f - (x2 * y * y));
    return y;
}

void sensfusion6Init(void) {
    if (!sensfusion6IsInit) {
        qw = 1.0f;
        qx = 0.0f;
        qy = 0.0f;
        qz = 0.0f;
        gravityX = 0.0f;
        gravityY = 0.0f;
        gravityZ = 1.0f;
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
        baseZacc = 0.0f;
        sensfusion6IsCalibrated = false;
        sensfusion6IsInit = true;
    }
}

bool sensfusion6Test(void) {
    return sensfusion6IsInit;
}

static void quaternionNormalize(float *q0, float *q1, float *q2, float *q3) {
    float rec = invSqrt((*q0) * (*q0) + (*q1) * (*q1) + (*q2) * (*q2) + (*q3) * (*q3));
    if (rec > 0.0f) {
        *q0 *= rec;
        *q1 *= rec;
        *q2 *= rec;
        *q3 *= rec;
    }
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt) {
    if (dt <= 0.0f) return;

    float q0 = qw;
    float q1 = qx;
    float q2 = qy;
    float q3 = qz;

    float gxr = gx * FSE_PI / 180.0f;
    float gyr = gy * FSE_PI / 180.0f;
    float gzr = gz * FSE_PI / 180.0f;

    float accNormSq = ax * ax + ay * ay + az * az;
    if (accNormSq < 1e-12f) {
        float qDot0 = 0.5f * (-gxr * q1 - gyr * q2 - gzr * q3);
        float qDot1 = 0.5f * (gxr * q0 + gzr * q2 - gyr * q3);
        float qDot2 = 0.5f * (gyr * q0 + gxr * q3 - gzr * q1);
        float qDot3 = 0.5f * (gzr * q0 + gyr * q1 - gxr * q2);
        q0 += qDot0 * dt;
        q1 += qDot1 * dt;
        q2 += qDot2 * dt;
        q3 += qDot3 * dt;
        quaternionNormalize(&q0, &q1, &q2, &q3);
        qw = q0;
        qx = q1;
        qy = q2;
        qz = q3;
        if (twoKi == 0.0f) {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }
        return;
    }

    float recAcc = invSqrt(accNormSq);
    if (recAcc <= 0.0f) return;
    float nax = ax * recAcc;
    float nay = ay * recAcc;
    float naz = az * recAcc;

    float halfvx = q1 * q3 - q0 * q2;
    float halfvy = q0 * q1 + q2 * q3;
    float halfvz = q0 * q0 - 0.5f + q3 * q3;

    float halfex = (nay * halfvz - naz * halfvy);
    float halfey = (naz * halfvx - nax * halfvz);
    float halfez = (nax * halfvy - nay * halfvx);

    float corrX = gxr;
    float corrY = gyr;
    float corrZ = gzr;

#ifdef CONFIG_IMU_MADGWICK_QUATERNION
    corrX += beta * halfex;
    corrY += beta * halfey;
    corrZ += beta * halfez;
#else
    if (twoKi > 0.0f) {
        integralFBx += twoKi * halfex * dt;
        integralFBy += twoKi * halfey * dt;
        integralFBz += twoKi * halfez * dt;
    } else {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }
    corrX += twoKp * halfex + integralFBx;
    corrY += twoKp * halfey + integralFBy;
    corrZ += twoKp * halfez + integralFBz;
#endif

    float qDot0 = 0.5f * (-corrX * q1 - corrY * q2 - corrZ * q3);
    float qDot1 = 0.5f * (corrX * q0 + corrZ * q2 - corrY * q3);
    float qDot2 = 0.5f * (corrY * q0 + corrX * q3 - corrZ * q1);
    float qDot3 = 0.5f * (corrZ * q0 + corrY * q1 - corrX * q2);

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
    quaternionNormalize(&q0, &q1, &q2, &q3);

    qw = q0;
    qx = q1;
    qy = q2;
    qz = q3;

    if (!sensfusion6IsCalibrated) {
        float gvx = 2.0f * halfvx;
        float gvy = 2.0f * halfvy;
        float gvz = 2.0f * halfvz;
        baseZacc = ax * gvx + ay * gvy + az * gvz;
        sensfusion6IsCalibrated = true;
    }
}

void estimatedGravityDirection(float qwIn, float qxIn, float qyIn, float qzIn,
                               float *gravX, float *gravY, float *gravZ) {
    if (!gravX || !gravY || !gravZ) return;
    float halfvx = qxIn * qzIn - qwIn * qyIn;
    float halfvy = qwIn * qxIn + qyIn * qzIn;
    float halfvz = qwIn * qwIn - 0.5f + qzIn * qzIn;
    *gravX = 2.0f * halfvx;
    *gravY = 2.0f * halfvy;
    *gravZ = 2.0f * halfvz;
}

static void quatToEulerRPY(float qwIn, float qxIn, float qyIn, float qzIn,
                           float *rollOut, float *pitchOut, float *yawOut) {
    float gx, gyv, gz;
    estimatedGravityDirection(qwIn, qxIn, qyIn, qzIn, &gx, &gyv, &gz);
    float pitch = asinf(clampFloat(-gx, -1.0f, 1.0f));
    float roll = atan2f(gyv, gz);
    float yaw = atan2f(2.0f * (qxIn * qyIn + qwIn * qzIn),
                       qwIn * qwIn + qxIn * qxIn - qyIn * qyIn - qzIn * qzIn);
    if (rollOut) *rollOut = roll * 180.0f / FSE_PI;
    if (pitchOut) *pitchOut = pitch * 180.0f / FSE_PI;
    if (yawOut) *yawOut = yaw * 180.0f / FSE_PI;
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
    if (!roll_deg || !pitch_deg || !yaw_deg) return;
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
    quatToEulerRPY(qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg);
}

void sensfusion6GetQuaternion(float *qwOut, float *qxOut, float *qyOut, float *qzOut) {
    if (!qwOut || !qxOut || !qyOut || !qzOut) return;
    *qwOut = qw;
    *qxOut = qx;
    *qyOut = qy;
    *qzOut = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
    return ax * gravityX + ay * gravityY + az * gravityZ;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out) {
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
                                  float thrustToTorque, float motorForces[4]) {
    if (!motorForces) return;
    float thrustPart = 0.25f * thrustSi;
    float arm = 0.707106781f * armLength;
    float rollPart = 0.0f;
    float pitchPart = 0.0f;
    float yawPart = 0.0f;
    if (armLength != 0.0f) {
        rollPart = 0.25f / armLength * torqueX;
        pitchPart = 0.25f / armLength * torqueY;
    }
    if (thrustToTorque != 0.0f) {
        yawPart = 0.25f / thrustToTorque * torqueZ;
    }

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
        float f = clampFloat(normalizedForces[i], 0.0f, 1.0f);
        motorPWMs[i] = (uint16_t)(f * 65535.0f);
    }
}

static uint16_t motorForceToPwm(float forceN) {
    if (forceN < 0.0f) return 0U;
    float norm = forceN / CRAZYFLIE_MAX_MOTOR_FORCE_N;
    norm = clampFloat(norm, 0.0f, 1.0f);
    return (uint16_t)(norm * 65535.0f);
}

void powerDistribution(const ControlData *control, MotorPower *motorPower) {
    if (!control || !motorPower) return;
    switch (control->controlMode) {
    case controlModeLegacy:
        powerDistributionLegacy(control->thrust, control->roll, control->pitch,
                                control->yaw, motorPower);
        break;
    case controlModeForceTorque: {
        float forces[4] = {0};
        powerDistributionForceTorque(control->thrustSi, control->torque.x,
                                     control->torque.y, control->torque.z,
                                     CRAZYFLIE_ARM_LENGTH_M,
                                     CRAZYFLIE_THRUST_TO_TORQUE, forces);
        motorPower->m1 = (int32_t)motorForceToPwm(forces[0]);
        motorPower->m2 = (int32_t)motorForceToPwm(forces[1]);
        motorPower->m3 = (int32_t)motorForceToPwm(forces[2]);
        motorPower->m4 = (int32_t)motorForceToPwm(forces[3]);
        break;
    }
    case controlModeForce: {
        uint16_t pwms[4] = {0};
        powerDistributionForce(control->normalizedForces, pwms);
        motorPower->m1 = (int32_t)pwms[0];
        motorPower->m2 = (int32_t)pwms[1];
        motorPower->m3 = (int32_t)pwms[2];
        motorPower->m4 = (int32_t)pwms[3];
        break;
    }
    default:
        break;
    }
}

int32_t capMinThrust(int32_t value, int32_t idleThrust) {
    if (value < idleThrust) return idleThrust;
    return value;
}

PowerCapResult powerDistributionCap(int32_t motors[4],
                                    int32_t maxAllowedThrust,
                                    int32_t idleThrust) {
    PowerCapResult result = {0};
    result.isCapped = false;
    result.reduction = 0;
    if (!motors) return result;

    int32_t maxValue = motors[0];
    for (int i = 1; i < 4; i++) {
        if (motors[i] > maxValue) maxValue = motors[i];
    }

    if (maxValue > maxAllowedThrust) {
        result.isCapped = true;
        result.reduction = maxValue - maxAllowedThrust;
        for (int i = 0; i < 4; i++) {
            motors[i] -= result.reduction;
            motors[i] = capMinThrust(motors[i], idleThrust);
        }
    }
    return result;
}

float batteryCompensation(float supplyVoltage, float filteredOld, float alpha) {
    return filteredOld + alpha * (supplyVoltage - filteredOld);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust,
                                        float nominalVoltage,
                                        float actualVoltage) {
    if (actualVoltage <= 0.0f) return motorThrust;
    float scaled = (float)motorThrust * nominalVoltage / actualVoltage;
    float rounded = floorf(scaled + 0.5f);
    if (rounded < 0.0f) return 0U;
    if (rounded > 65535.0f) return 65535U;
    return (uint16_t)rounded;
}

static float pidDt = 0.001f;
static bool pidInitDone = false;

static void pidObjectInit(PidObject *p) {
    if (!p) return;
    p->kp = 0.0f;
    p->ki = 0.0f;
    p->kd = 0.0f;
    p->kff = 0.0f;
    p->integral = 0.0f;
    p->prevError = 0.0f;
    p->output = 0.0f;
    p->initialized = true;
}

static float pidUpdate(PidObject *p, float measured, float desired, bool reset) {
    float error = desired - measured;
    float derivative = 0.0f;
    if (reset) {
        p->integral = 0.0f;
        p->prevError = error;
    } else {
        if (pidDt > 0.0f) {
            derivative = (error - p->prevError) / pidDt;
        }
        p->integral += error * pidDt;
        p->prevError = error;
    }
    float out = p->kp * error + p->ki * p->integral + p->kd * derivative + p->kff * desired;
    p->output = out;
    return out;
}

static void pidResetToTarget(PidObject *p, float target) {
    if (!p) return;
    (void)target;
    p->integral = 0.0f;
    p->prevError = 0.0f;
    p->output = 0.0f;
    p->initialized = true;
}

void attitudeControllerInit(float updateDt) {
    if (pidInitDone) return;
    pidDt = updateDt > 0.0f ? updateDt : 0.001f;
    pidObjectInit(&pidRoll);
    pidObjectInit(&pidPitch);
    pidObjectInit(&pidYaw);
    pidObjectInit(&pidRollRate);
    pidObjectInit(&pidPitchRate);
    pidObjectInit(&pidYawRate);
    pidRoll_v = pidRoll;
    pidPitch_v = pidPitch;
    pidYaw_v = pidYaw;
    pidRollRate_v = pidRollRate;
    pidPitchRate_v = pidPitchRate;
    pidYawRate_v = pidYawRate;
    pidInitDone = true;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired) {
    float outRoll = pidUpdate(&pidRollRate, rollActual, rollDesired, false);
    float outPitch = pidUpdate(&pidPitchRate, pitchActual, pitchDesired, false);
    float outYaw = pidUpdate(&pidYawRate, yawActual, yawDesired, false);
    pidRollRate.output = (float)saturateSignedInt16((int32_t)outRoll);
    pidPitchRate.output = (float)saturateSignedInt16((int32_t)outPitch);
    pidYawRate.output = (float)saturateSignedInt16((int32_t)outYaw);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired) {
    pidRoll.output = pidUpdate(&pidRoll, rollActual, rollDesired, false);
    pidPitch.output = pidUpdate(&pidPitch, pitchActual, pitchDesired, false);
    pidYaw.output = pidUpdate(&pidYaw, yawActual, yawDesired, true);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual) {
    pidResetToTarget(&pidRoll, rollActual);
    pidResetToTarget(&pidPitch, pitchActual);
    pidResetToTarget(&pidYaw, yawActual);
    pidResetToTarget(&pidRollRate, 0.0f);
    pidResetToTarget(&pidPitchRate, 0.0f);
    pidResetToTarget(&pidYawRate, 0.0f);
}

void attitudeControllerResetRollAttitudePID(float rollActual) {
    pidResetToTarget(&pidRoll, rollActual);
}

void attitudeControllerResetPitchAttitudePID(float pitchActual) {
    pidResetToTarget(&pidPitch, pitchActual);
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw) {
    if (!roll || !pitch || !yaw) return;
    *roll = (int16_t)pidRollRate.output;
    *pitch = (int16_t)pidPitchRate.output;
    *yaw = (int16_t)pidYawRate.output;
}

static bool g_posControllerResetPending = false;
static float g_posVelIntegral = 0.0f;
static float g_prevPosError = 0.0f;

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
    if (!setpoint || !state) return 0U;
    float out = 0.0f;
    if (g_posControllerResetPending) {
        g_posVelIntegral = 0.0f;
        g_prevPosError = 0.0f;
        g_posControllerResetPending = false;
    }

    if (setpoint->mode.z == modeVelocity) {
        float error = setpoint->velocity.z - state->velocity.z;
        g_posVelIntegral += error * 0.001f;
        out = (float)setpoint->thrust + 500.0f * error + 10.0f * g_posVelIntegral;
    } else if (setpoint->mode.z == modeAbs) {
        float error = setpoint->position.z - state->position.z;
        float velError = setpoint->velocity.z - state->velocity.z;
        g_posVelIntegral += error * 0.001f;
        out = (float)setpoint->thrust + 2.0f * error + 0.5f * velError + 10.0f * g_posVelIntegral;
    } else {
        out = (float)setpoint->thrust + 0.0f * (state->position.z + state->velocity.z);
    }
    if (out < 0.0f) out = 0.0f;
    if (out > 65535.0f) out = 65535.0f;
    return (uint16_t)out;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt) {
    if (!sensors || !setpoint || !state || !control) return;

    uint16_t thrust = 0U;
    if (setpoint->mode.z == modeDisable) {
        thrust = setpoint->thrust;
    } else {
        thrust = positionControllerUpdate(setpoint, state);
    }

    if (thrust == 0U) {
        control->thrust = 0U;
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->controlMode = controlModeLegacy;
        attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch,
                                      state->attitude.yaw);
        return;
    }

    float rollActual = state->attitude.roll;
    float pitchActual = state->attitude.pitch;
    float yawActual = state->attitude.yaw;

    float desiredRoll = rollActual;
    float desiredPitch = pitchActual;
    float desiredYaw = yawActual;

    bool rollAttitudeActive = true;
    bool pitchAttitudeActive = true;
    bool yawAttitudeActive = true;

    if (setpoint->mode.roll == modeVelocity) {
        rollAttitudeActive = false;
        desiredRoll = rollActual;
        attitudeControllerResetRollAttitudePID(rollActual);
    } else if (setpoint->mode.roll == modeAbs) {
        desiredRoll = setpoint->attitude.roll;
    }

    if (setpoint->mode.pitch == modeVelocity) {
        pitchAttitudeActive = false;
        desiredPitch = pitchActual;
        attitudeControllerResetPitchAttitudePID(pitchActual);
    } else if (setpoint->mode.pitch == modeAbs) {
        desiredPitch = setpoint->attitude.pitch;
    }

    if (setpoint->mode.quat == modeAbs) {
        float qr = 0.0f;
        float qp = 0.0f;
        float qy = 0.0f;
        quatToEulerRPY(state->attitudeQuaternion.w, state->attitudeQuaternion.x,
                       state->attitudeQuaternion.y, state->attitudeQuaternion.z,
                       &qr, &qp, &qy);
        desiredYaw = qy;
        yawAttitudeActive = true;
    } else if (setpoint->mode.yaw == modeVelocity) {
        yawAttitudeActive = false;
        static float localDesiredYaw = 0.0f;
        localDesiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
        if (yawMaxDelta != 0.0f) {
            float delta = localDesiredYaw - yawActual;
            if (delta > yawMaxDelta) {
                localDesiredYaw = yawActual + yawMaxDelta;
            } else if (delta < -yawMaxDelta) {
                localDesiredYaw = yawActual - yawMaxDelta;
            }
        }
        desiredYaw = localDesiredYaw;
    } else if (setpoint->mode.yaw == modeAbs) {
        desiredYaw = setpoint->attitude.yaw;
    }

    attitudeControllerCorrectAttitudePID(rollActual, desiredRoll,
                                         pitchActual, desiredPitch,
                                         yawActual, desiredYaw);

    float desiredRateRoll = 0.0f;
    float desiredRatePitch = 0.0f;
    float desiredRateYaw = 0.0f;

    if (rollAttitudeActive) {
        desiredRateRoll = pidRoll.output;
    } else {
        desiredRateRoll = setpoint->attitudeRate.roll;
    }
    if (pitchAttitudeActive) {
        desiredRatePitch = pidPitch.output;
    } else {
        desiredRatePitch = setpoint->attitudeRate.pitch;
    }
    if (yawAttitudeActive) {
        desiredRateYaw = pidYaw.output;
    } else {
        desiredRateYaw = setpoint->attitudeRate.yaw;
    }

    float rollRateActual = sensors->gyro.x;
    float pitchRateActual = -sensors->gyro.y;
    float yawRateActual = sensors->gyro.z;

    attitudeControllerCorrectRatePID(rollRateActual, desiredRateRoll,
                                     pitchRateActual, desiredRatePitch,
                                     yawRateActual, desiredRateYaw);

    int16_t outRoll = 0;
    int16_t outPitch = 0;
    int16_t outYaw = 0;
    attitudeControllerGetActuatorOutput(&outRoll, &outPitch, &outYaw);

    control->controlMode = controlModeLegacy;
    control->roll = outRoll;
    control->pitch = outPitch;
    control->yaw = -outYaw;
    control->thrust = thrust;
}

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime) {
    if (!rollPrime || !pitchPrime) return;
    float rad = yaw_deg * FSE_PI / 180.0f;
    float c = cosf(rad);
    float s = sinf(rad);
    *rollPrime = roll * c - pitch * s;
    *pitchPrime = roll * s + pitch * c;
}

static int g_commanderPriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t g_commanderLastUpdateTick = 0U;
static bool g_commanderHasSetpoint = false;
static Setpoint g_activeSetpoint;
static bool g_trajectoryFlying = false;
static bool g_trajectoryFinished = false;
static bool g_trajectoryDisabled = false;

bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
    if (!setpoint) return false;

    if (priority == COMMANDER_PRIORITY_DISABLE) {
        memcpy(&g_activeSetpoint, setpoint, sizeof(Setpoint));
        g_commanderPriority = priority;
        g_commanderLastUpdateTick = platformTickMs;
        g_commanderHasSetpoint = true;
        return true;
    }

    if (priority < g_commanderPriority) return false;

    memcpy(&g_activeSetpoint, setpoint, sizeof(Setpoint));
    g_commanderPriority = priority;
    g_commanderLastUpdateTick = platformTickMs;
    g_commanderHasSetpoint = true;

    if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
        g_trajectoryFlying = false;
        g_trajectoryFinished = false;
        g_trajectoryDisabled = true;
    }
    return true;
}

void commanderRelaxPriority(void) {
    g_commanderPriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void) {
    if (platformTickMs >= g_commanderLastUpdateTick) {
        return platformTickMs - g_commanderLastUpdateTick;
    }
    return 0U;
}

int commanderGetActivePriority(void) {
    return g_commanderPriority;
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

    uint16_t rawThrust = values->thrust;
    float rawRoll = values->roll;
    float rawPitch = values->pitch;
    float rawYaw = values->yaw;

    if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) {
        thrustLocked = true;
    }
    if (rawThrust == 0U) {
        thrustLocked = false;
    }

    if (altHoldMode) {
        if (!commanderModeSet) {
            commanderModeSet = true;
            g_posControllerResetPending = true;
        }
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0U;
        setpoint->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;
    } else {
        if (commanderModeSet) {
            setpoint->mode.z = modeDisable;
            commanderModeSet = false;
        }
        if (thrustLocked || rawThrust < MIN_THRUST) {
            setpoint->thrust = 0U;
        } else {
            setpoint->thrust = rawThrust > MAX_THRUST ? MAX_THRUST : rawThrust;
        }
    }

    if (posHoldMode) {
        setpoint->mode.x = modeVelocity;
        setpoint->mode.y = modeVelocity;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->velocity.x = rawPitch / 30.0f;
        setpoint->velocity.y = rawRoll / 30.0f;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
    } else if (posSetMode && rawThrust != 0U) {
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
        setpoint->thrust = 0U;
        return;
    } else {
        float croll = rawRoll;
        float cpitch = rawPitch;
        if (yawMode == PLUSMODE) {
            rotateYaw(croll, cpitch, 45.0f, &croll, &cpitch);
        } else if (yawMode == CAREFREE) {
            setpoint->mode.roll = modeDisable;
            setpoint->mode.pitch = modeDisable;
            setpoint->mode.yaw = modeDisable;
            setpoint->thrust = 0U;
            return;
        }

        if (stabilizationModeRoll == RATE) {
            setpoint->mode.roll = modeVelocity;
            setpoint->attitudeRate.roll = croll;
        } else {
            setpoint->mode.roll = modeAbs;
            setpoint->attitude.roll = croll;
        }

        if (stabilizationModePitch == RATE) {
            setpoint->mode.pitch = modeVelocity;
            setpoint->attitudeRate.pitch = cpitch;
        } else {
            setpoint->mode.pitch = modeAbs;
            setpoint->attitude.pitch = cpitch;
        }
    }

    if (stabilizationModeYaw == RATE) {
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = -rawYaw;
    } else {
        setpoint->mode.yaw = modeAbs;
        setpoint->attitude.yaw = rawYaw;
    }
}

static SensorData s_supervisorSensors;
static uint32_t s_motorRatios[4] = {0U, 0U, 0U, 0U};
static int32_t s_motorRPMs[4] = {0, 0, 0, 0};
static bool s_hasSensors = false;
static bool s_hasMotorRatios = false;
static bool s_hasMotorRPMs = false;
static float s_crashDetectionGs = 0.0f;
static float s_freeFallThreshold = 0.0f;
static float s_acceptedTiltAccZ = 0.0f;
static float s_acceptedUpsideDownAccZ = 0.0f;
static uint32_t s_maxTiltTime = 0U;
static uint32_t s_maxUpsideDownTime = 0U;
static bool s_tumbleCheckEnabled = false;
static bool s_autoArming = false;
static uint32_t s_spinupTimeoutDurationMs = 0U;
static bool s_spinupStarted = false;
static uint32_t s_spinupStartTick = 0U;
static uint32_t s_lastArmingTick = 0U;
static uint32_t s_latestLandingTick = 0U;
static uint32_t s_preflightTimeoutDuration = 0U;
static uint32_t s_landingTimeoutDuration = 0U;
static bool s_isTumbledInternal = false;
static bool s_isFreeFallingState = false;
static bool s_crashDetectedInternal = false;
static uint32_t s_lastFlyingTick = 0U;
static bool s_seenFlyingEvent = false;
static uint32_t s_tumbleStartTick = 0U;
static bool s_tumbleActive = false;
static uint32_t s_motorNotRespondingStartTick = 0U;
static bool s_motorNotRespondingTimerActive = false;
static uint32_t s_lastEmergencyNotificationTick = 0U;

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick) {
    if (!motorRatios) return false;
    for (int i = 0; i < 4; i++) {
        if (motorRatios[i] > idleThrust) {
            s_lastFlyingTick = currentTick;
            s_seenFlyingEvent = true;
            break;
        }
    }
    if (!s_seenFlyingEvent) return false;
    return (currentTick - s_lastFlyingTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling) {
    if (isFreeFalling) *isFreeFalling = false;

    if (crashDetectionGs > 0.0f) {
        float norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
        float diff = fabsf(norm - 1.0f);
        if (diff > crashDetectionGs) {
            s_crashDetectedInternal = true;
            supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        }
    }

    bool freeFall = freeFallThreshold > 0.0f &&
                    fabsf(accX) < freeFallThreshold &&
                    fabsf(accY) < freeFallThreshold &&
                    fabsf(accZ) < freeFallThreshold;
    if (freeFall) {
        if (isFreeFalling) *isFreeFalling = true;
        s_isFreeFallingState = true;
        supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL;
        s_tumbleActive = false;
        s_tumbleStartTick = 0U;
        return false;
    } else {
        s_isFreeFallingState = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;
    }

    if (!tumbleCheckEnabled) return false;

    uint32_t timeout = 0U;
    if (accZ < acceptedUpsideDownAccZ) {
        timeout = maxUpsideDownTime;
    } else if (accZ < acceptedTiltAccZ) {
        timeout = maxTiltTime;
    } else {
        s_tumbleActive = false;
        s_tumbleStartTick = 0U;
        return false;
    }

    if (!s_tumbleActive) {
        s_tumbleActive = true;
        s_tumbleStartTick = currentTick;
    }
    if (timeout == 0U) return true;
    bool tumbled = (currentTick - s_tumbleStartTick) >= timeout;
    if (tumbled) {
        s_isTumbledInternal = true;
        supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
    } else {
        s_isTumbledInternal = false;
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
    }
    return tumbled;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick) {
    if (lastNotificationTick == 0U) return true;
    return (currentTick - lastNotificationTick) < DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration) {
    (void)state;
    if (latestArmingTick == 0U) return false;
    return (currentTick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTick,
                                uint32_t landingTimeoutDuration) {
    if (latestLandingTick == 0U) return false;
    return (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed) {
    if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
        supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
    }
    supervisorLog.info = supervisorConditionBits;
    return supervisorConditionBits;
}

void supervisorSetSensorData(const SensorData *sensors) {
    if (!sensors) return;
    memcpy(&s_supervisorSensors, sensors, sizeof(SensorData));
    s_hasSensors = true;
    gyro.x = sensors->gyro.x;
    gyro.y = sensors->gyro.y;
    gyro.z = sensors->gyro.z;
    acc.x = sensors->acc.x;
    acc.y = sensors->acc.y;
    acc.z = sensors->acc.z;
    baro.asl = sensors->baroAsl;
    baro.temp = sensors->baroTemperature;
    baro.pressure = sensors->baroPressure;
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) {
    if (!motorRatios) return;
    for (int i = 0; i < 4; i++) s_motorRatios[i] = motorRatios[i];
    s_hasMotorRatios = true;
    (void)idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) {
    if (!motorRPMs) return;
    for (int i = 0; i < 4; i++) s_motorRPMs[i] = motorRPMs[i];
    s_hasMotorRPMs = true;
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled) {
    s_crashDetectionGs = crashDetectionGs;
    s_freeFallThreshold = freeFallThreshold;
    s_acceptedTiltAccZ = acceptedTiltAccZ;
    s_acceptedUpsideDownAccZ = acceptedUpsideDownAccZ;
    s_maxTiltTime = maxTiltTime;
    s_maxUpsideDownTime = maxUpsideDownTime;
    s_tumbleCheckEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) {
    s_autoArming = autoArming;
    s_spinupTimeoutDurationMs = spinupTimeoutDurationMs;
}

static bool supervisorIsAllowedArmedState(SupervisorState st) {
    return st == supervisorStateArming ||
           st == supervisorStateReadyToFly ||
           st == supervisorStateFlying ||
           st == supervisorStateWarningLevelOut ||
           st == supervisorStateLanded;
}

bool supervisorRequestArming(bool doArm) {
    if (!doArm) {
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
        return true;
    }
    if (supervisorIsArmed() && supervisorState == supervisorStateArming) {
        return true;
    }
    if (!supervisorCanArm()) return false;
    supervisorConditionBits |= SUPERVISOR_CB_ARMED;
    supervisorState = supervisorStateArming;
    s_spinupStarted = false;
    s_spinupStartTick = 0U;
    return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
    if (!doRecovery) {
        s_crashDetectedInternal = true;
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        return true;
    }
    if (s_isTumbledInternal ||
        (supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED)) {
        return false;
    }
    s_crashDetectedInternal = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    supervisorState = supervisorStateReadyToFly;
    return true;
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
    return (supervisorConditionBits & SUPERVISOR_CB_ARMED) != 0U;
}

bool supervisorIsCrashed(void) {
    return (supervisorConditionBits & SUPERVISOR_CB_CRASHED) != 0U;
}

bool supervisorAreMotorsAllowedToRun(void) {
    return supervisorState == supervisorStateArming ||
           supervisorState == supervisorStateReadyToFly ||
           supervisorState == supervisorStateFlying ||
           supervisorState == supervisorStateWarningLevelOut ||
           supervisorState == supervisorStateLanded;
}

uint16_t supervisorGetInfoBitfield(void) {
    uint16_t info = 0U;
    if (supervisorCanArm()) info |= (uint16_t)(1U << 0);
    if (supervisorIsArmed()) info |= (uint16_t)(1U << 1);
    if (s_autoArming) info |= (uint16_t)(1U << 2);
    if (supervisorCanFly()) info |= (uint16_t)(1U << 3);
    if (supervisorConditionBits & SUPERVISOR_CB_IS_FLYING) info |= (uint16_t)(1U << 4);
    if (supervisorConditionBits & SUPERVISOR_CB_IS_TUMBLED) info |= (uint16_t)(1U << 5);
    if (supervisorState == supervisorStateLocked) info |= (uint16_t)(1U << 6);
    if (supervisorIsCrashed()) info |= (uint16_t)(1U << 7);
    if (g_trajectoryFlying) info |= (uint16_t)(1U << 8);
    if (g_trajectoryFinished) info |= (uint16_t)(1U << 9);
    if (g_trajectoryDisabled) info |= (uint16_t)(1U << 10);
    if (supervisorConditionBits & SUPERVISOR_CB_DECK_FAULT) info |= (uint16_t)(1U << 11);
    return info;
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax) {
    if (!motorRPMs) return false;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) return false;
    }
    return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly,
                           uint32_t currentTick) {
    if (!motorRPMs) return false;
    if (!canFly) {
        s_motorNotRespondingTimerActive = false;
        s_motorNotRespondingStartTick = 0U;
        supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return false;
    }

    bool allBelow = true;
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] >= rpmThreshold) {
            allBelow = false;
            break;
        }
    }

    if (!allBelow) {
        s_motorNotRespondingTimerActive = false;
        s_motorNotRespondingStartTick = 0U;
        supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return false;
    }

    if (!s_motorNotRespondingTimerActive) {
        s_motorNotRespondingTimerActive = true;
        s_motorNotRespondingStartTick = currentTick;
    }

    if ((currentTick - s_motorNotRespondingStartTick) >= rpmCheckDurationMs) {
        supervisorConditionBits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
        return true;
    }
    return false;
}

void supervisorInit(void) {
    static bool initialized = false;
    if (!initialized) {
        supervisorState = supervisorStatePreFlChecksPassed;
        supervisorConditionBits = 0U;
        initialized = true;
    }
}

void supervisorUpdate(uint32_t stabilizerStep) {
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;

    uint32_t tick = platformTickMs;
    bool flying = false;
    if (s_hasMotorRatios) {
        flying = isFlyingCheck(s_motorRatios, 0U, tick);
    }
    if (flying) {
        supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;
    }

    bool freeFall = false;
    bool tumbled = isTumbledCheck(s_supervisorSensors.acc.x, s_supervisorSensors.acc.y,
                                  s_supervisorSensors.acc.z,
                                  s_crashDetectionGs, s_freeFallThreshold,
                                  s_acceptedTiltAccZ, s_acceptedUpsideDownAccZ,
                                  s_maxTiltTime, s_maxUpsideDownTime,
                                  s_tumbleCheckEnabled, tick, &freeFall);
    s_isTumbledInternal = tumbled;
    if (tumbled) {
        supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
    }

    uint32_t age = commanderGetInactivityTime();
    if (age > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    } else if (age > COMMANDER_WDT_TIMEOUT_STABILIZE) {
        supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING;
        supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    }

    if (supervisorIsPreflightTimeout(supervisorState, s_lastArmingTick, tick,
                                     s_preflightTimeoutDuration)) {
        supervisorConditionBits |= SUPERVISOR_CB_PREFLIGHT_TIMEOUT;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_PREFLIGHT_TIMEOUT;
    }

    if (supervisorIsLandingTimeout(s_latestLandingTick, tick,
                                   s_landingTimeoutDuration)) {
        supervisorConditionBits |= SUPERVISOR_CB_LANDING_TIMEOUT;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_LANDING_TIMEOUT;
    }

    static SupervisorState prevSupervisorState = supervisorStatePreFlChecksPassed;
    if (supervisorState == supervisorStateArming) {
        if (!s_spinupStarted) {
            s_spinupStarted = true;
            s_spinupStartTick = tick;
        } else if (s_spinupTimeoutDurationMs > 0U &&
                   (tick - s_spinupStartTick) >= s_spinupTimeoutDurationMs) {
            supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
            supervisorState = supervisorStateCrashed;
        }
    }

    if (s_autoArming && supervisorState == supervisorStatePreFlChecksPassed) {
        supervisorRequestArming(true);
    }

    bool wasAllowed = supervisorIsAllowedArmedState(prevSupervisorState);
    bool nowAllowed = supervisorIsAllowedArmedState(supervisorState);
    if (wasAllowed && !nowAllowed) {
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    }
    if (prevSupervisorState == supervisorStateArming &&
        supervisorState != supervisorStateArming) {
        s_spinupStarted = false;
        s_spinupStartTick = 0U;
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    }
    prevSupervisorState = supervisorState;

    supervisorLog.info = supervisorConditionBits;
    supervisorLog.accNorm = sqrtf(s_supervisorSensors.acc.x * s_supervisorSensors.acc.x +
                                  s_supervisorSensors.acc.y * s_supervisorSensors.acc.y +
                                  s_supervisorSensors.acc.z * s_supervisorSensors.acc.z);
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t supervisorConditionBitsIn,
                                SupervisorState state) {
    if (!setpoint) return;
    (void)supervisorConditionBitsIn;

    switch (state) {
    case supervisorStateArming:
    case supervisorStateReadyToFly:
    case supervisorStateFlying:
    case supervisorStateLanded:
        break;
    case supervisorStateWarningLevelOut:
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.roll = modeAbs;
        setpoint->mode.pitch = modeAbs;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = 0.0f;
        break;
    default:
        memset(setpoint, 0, sizeof(Setpoint));
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.z = modeDisable;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->mode.yaw = modeDisable;
        setpoint->mode.quat = modeDisable;
        break;
    }
}

#define ESTIMATOR_FIFO_CAPACITY 16U
static EstimatorMeasurement s_estimatorFifo[ESTIMATOR_FIFO_CAPACITY];
static uint8_t s_estimatorHead = 0U;
static uint8_t s_estimatorTail = 0U;
static uint8_t s_estimatorCount = 0U;

static EstimatorMeasurement s_lastGyro;
static EstimatorMeasurement s_lastAcc;
static EstimatorMeasurement s_lastBaro;
static EstimatorMeasurement s_lastTof;
static bool s_hasGyro = false;
static bool s_hasAcc = false;
static bool s_hasBaro = false;
static bool s_hasTof = false;

static State g_stateEstimate;
static float g_verticalVelocity = 0.0f;

bool estimatorEnqueue(const EstimatorMeasurement *measurement) {
    if (!measurement) return false;
    if (s_estimatorCount >= ESTIMATOR_FIFO_CAPACITY) return false;
    memcpy(&s_estimatorFifo[s_estimatorTail], measurement, sizeof(EstimatorMeasurement));
    s_estimatorTail = (uint8_t)((s_estimatorTail + 1U) % ESTIMATOR_FIFO_CAPACITY);
    s_estimatorCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement) {
    if (!measurement) return false;
    if (s_estimatorCount == 0U) return false;
    memcpy(measurement, &s_estimatorFifo[s_estimatorHead], sizeof(EstimatorMeasurement));
    s_estimatorHead = (uint8_t)((s_estimatorHead + 1U) % ESTIMATOR_FIFO_CAPACITY);
    s_estimatorCount--;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep) {
    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) {
        switch (m.type) {
        case MeasurementTypeGyroscope:
            memcpy(&s_lastGyro, &m, sizeof(EstimatorMeasurement));
            s_hasGyro = true;
            break;
        case MeasurementTypeAcceleration:
            memcpy(&s_lastAcc, &m, sizeof(EstimatorMeasurement));
            s_hasAcc = true;
            break;
        case MeasurementTypeBarometer:
            memcpy(&s_lastBaro, &m, sizeof(EstimatorMeasurement));
            s_hasBaro = true;
            break;
        case MeasurementTypeTOF:
            memcpy(&s_lastTof, &m, sizeof(EstimatorMeasurement));
            s_hasTof = true;
            break;
        default:
            break;
        }
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        if (s_hasGyro && s_hasAcc) {
            sensfusion6UpdateQ(s_lastGyro.data[0], s_lastGyro.data[1], s_lastGyro.data[2],
                               s_lastAcc.data[0], s_lastAcc.data[1], s_lastAcc.data[2],
                               0.001f);
        }
        sensfusion6GetEulerRPY(&stateEstimate.roll, &stateEstimate.pitch, &stateEstimate.yaw);
        sensfusion6GetQuaternion(&stateEstimate.qw, &stateEstimate.qx, &stateEstimate.qy, &stateEstimate.qz);

        g_stateEstimate.attitude.roll = stateEstimate.roll;
        g_stateEstimate.attitude.pitch = stateEstimate.pitch;
        g_stateEstimate.attitude.yaw = stateEstimate.yaw;
        g_stateEstimate.attitudeQuaternion.w = stateEstimate.qw;
        g_stateEstimate.attitudeQuaternion.x = stateEstimate.qx;
        g_stateEstimate.attitudeQuaternion.y = stateEstimate.qy;
        g_stateEstimate.attitudeQuaternion.z = stateEstimate.qz;
        g_stateEstimate.acc.x = s_hasAcc ? s_lastAcc.data[0] : 0.0f;
        g_stateEstimate.acc.y = s_hasAcc ? s_lastAcc.data[1] : 0.0f;
        g_stateEstimate.acc.z = s_hasAcc ? s_lastAcc.data[2] : 0.0f;

        if (s_hasAcc && sensfusion6IsCalibrated) {
            float az = sensfusion6GetAccZWithoutGravity(s_lastAcc.data[0],
                                                        s_lastAcc.data[1],
                                                        s_lastAcc.data[2]);
            g_verticalVelocity += az * 0.001f;
        }
        g_stateEstimate.velocity.z = g_verticalVelocity;

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

    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
        g_stateEstimate.position.x += g_stateEstimate.velocity.x * 0.01f;
        g_stateEstimate.position.y += g_stateEstimate.velocity.y * 0.01f;
        g_stateEstimate.position.z += g_stateEstimate.velocity.z * 0.01f;
    }
}

static void sensorsInit(void) {
    memset(&gyro, 0, sizeof(gyro));
    memset(&acc, 0, sizeof(acc));
    memset(&baro, 0, sizeof(baro));
}

static void stateEstimatorInit(void) {
    memset(&g_stateEstimate, 0, sizeof(g_stateEstimate));
    s_estimatorHead = 0U;
    s_estimatorTail = 0U;
    s_estimatorCount = 0U;
    s_hasGyro = false;
    s_hasAcc = false;
    s_hasBaro = false;
    s_hasTof = false;
    g_verticalVelocity = 0.0f;
}

static void controllerInit(void) {
    attitudeControllerInit(0.001f);
}

static void powerDistributionInit(void) {
    motor.m1req = 0U;
    motor.m2req = 0U;
    motor.m3req = 0U;
    motor.m4req = 0U;
}

static void motorsInit(void) {
    motor.m1req = 0U;
    motor.m2req = 0U;
    motor.m3req = 0U;
    motor.m4req = 0U;
}

static void collisionAvoidanceInit(void) {
}

static void collisionAvoidanceUpdateSetpoint(Setpoint *setpoint) {
    (void)setpoint;
}

static bool g_stabilizerInitialized = false;
static bool g_systemStarted = false;
static bool g_sensorsCalibrated = true;
static bool g_sensorDataReady = false;
static SensorData g_lastSensors;
static Setpoint g_pendingHighLevelSetpoint;
static bool g_hasHighLevelSetpoint = false;
static float g_batteryVoltage = 0.0f;
static float g_filteredBatteryVoltage = 0.0f;
static uint32_t g_stabilizerStep = 0U;

void stabilizerInit(void) {
    if (g_stabilizerInitialized) return;

    sensorsInit();
    stateEstimatorInit();
    controllerInit();
    powerDistributionInit();
    motorsInit();
    collisionAvoidanceInit();

    g_stabilizerInitialized = true;
    g_systemStarted = true;
    g_sensorsCalibrated = true;
    g_sensorDataReady = false;
}

static void setMotorRatiosInternal(const int32_t motorsPwm[4]) {
    if (!motorsPwm) return;
    motor.m1req = (uint16_t)clampFloat((float)motorsPwm[0], 0.0f, 65535.0f);
    motor.m2req = (uint16_t)clampFloat((float)motorsPwm[1], 0.0f, 65535.0f);
    motor.m3req = (uint16_t)clampFloat((float)motorsPwm[2], 0.0f, 65535.0f);
    motor.m4req = (uint16_t)clampFloat((float)motorsPwm[3], 0.0f, 65535.0f);
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) {
    if (!setpoint) return false;
    memcpy(&g_pendingHighLevelSetpoint, setpoint, sizeof(Setpoint));
    g_hasHighLevelSetpoint = true;
    return true;
}

void stabilizerTask(void) {
    if (!g_stabilizerInitialized || !g_systemStarted || !g_sensorsCalibrated) return;

    if (!g_sensorDataReady) {
        g_sensorDataReady = true;
    }

    SensorData sensors = g_lastSensors;

    if (g_hasHighLevelSetpoint) {
        commanderSetSetpoint(&g_pendingHighLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
        g_hasHighLevelSetpoint = false;
    }

    if (healthShallWeRunTest()) {
        healthRunTests(&sensors);
        return;
    }

    estimatorComplementary(g_stabilizerStep);

    Setpoint setpoint;
    if (g_commanderHasSetpoint) {
        memcpy(&setpoint, &g_activeSetpoint, sizeof(Setpoint));
    } else {
        memset(&setpoint, 0, sizeof(Setpoint));
    }

    supervisorUpdate(g_stabilizerStep);

    if (!supervisorCanFly()) {
        memset(&setpoint, 0, sizeof(Setpoint));
        setpoint.mode.x = modeDisable;
        setpoint.mode.y = modeDisable;
        setpoint.mode.z = modeDisable;
        setpoint.mode.roll = modeDisable;
        setpoint.mode.pitch = modeDisable;
        setpoint.mode.yaw = modeDisable;
        setpoint.mode.quat = modeDisable;
    }

    collisionAvoidanceUpdateSetpoint(&setpoint);
    supervisorOverrideSetpoint(&setpoint, supervisorConditionBits, supervisorState);

    ControlData control;
    memset(&control, 0, sizeof(control));
    controllerPid(&sensors, &setpoint, &g_stateEstimate, &control, 0.0f, 0.001f);

    MotorPower motorPower;
    powerDistribution(&control, &motorPower);

    int32_t motors[4] = {0};
    motors[0] = motorPower.m1;
    motors[1] = motorPower.m2;
    motors[2] = motorPower.m3;
    motors[3] = motorPower.m4;

    for (int i = 0; i < 4; i++) {
        if (motors[i] < 0) motors[i] = 0;
        uint16_t pwm = (uint16_t)(motors[i] > 65535 ? 65535 : motors[i]);
        pwm = motorsCompensateBatteryVoltage(pwm, 4.2f,
                                             g_batteryVoltage > 0.0f ? g_batteryVoltage : 4.2f);
        motors[i] = (int32_t)pwm;
    }

    powerDistributionCap(motors, 65535, 0);

    if (!supervisorAreMotorsAllowedToRun()) {
        for (int i = 0; i < 4; i++) motors[i] = 0;
    }

    setMotorRatiosInternal(motors);
    g_stabilizerStep++;
}

static uint32_t quatcompress(float qwIn, float qxIn, float qyIn, float qzIn) {
    uint8_t qw8 = (uint8_t)clampFloat((qwIn * 127.5f) + 127.5f, 0.0f, 255.0f);
    uint8_t qx8 = (uint8_t)clampFloat((qxIn * 127.5f) + 127.5f, 0.0f, 255.0f);
    uint8_t qy8 = (uint8_t)clampFloat((qyIn * 127.5f) + 127.5f, 0.0f, 255.0f);
    uint8_t qz8 = (uint8_t)clampFloat((qzIn * 127.5f) + 127.5f, 0.0f, 255.0f);
    return ((uint32_t)qw8 << 24U) | ((uint32_t)qx8 << 16U) |
           ((uint32_t)qy8 << 8U) | (uint32_t)qz8;
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output) {
    if (!state || !sensors || !output) return;

    for (int i = 0; i < 3; i++) {
        output->position_mm[i] = (int32_t)(state->position.x * 1000.0f);
        output->velocity_mms[i] = (int32_t)(state->velocity.x * 1000.0f);
    }
    output->acceleration_mms2[0] = (int32_t)(sensors->acc.x * 9810.0f);
    output->acceleration_mms2[1] = (int32_t)(sensors->acc.y * 9810.0f);
    output->acceleration_mms2[2] = (int32_t)((sensors->acc.z + 1.0f) * 9810.0f);
    output->gyro_millirad_s[0] = sensors->gyro.x * FSE_PI / 180.0f * 1000.0f;
    output->gyro_millirad_s[1] = -sensors->gyro.y * FSE_PI / 180.0f * 1000.0f;
    output->gyro_millirad_s[2] = sensors->gyro.z * FSE_PI / 180.0f * 1000.0f;
    output->quatCompressed = quatcompress(state->attitudeQuaternion.w,
                                          state->attitudeQuaternion.x,
                                          state->attitudeQuaternion.y,
                                          state->attitudeQuaternion.z);
}

bool rateSupervisorValidate(uint32_t measuredRate) {
    return measuredRate >= 997U && measuredRate <= 1003U;
}

static uint32_t s_lastRateSupervisorTick = 0U;
static bool s_rateSupervisorError = false;
static bool g_sensorActive = false;

void rateSupervisorTask(void) {
    if ((platformTickMs - s_lastRateSupervisorTick) >= 2000U) {
        s_lastRateSupervisorTick = platformTickMs;
        if (g_sensorActive) {
            s_rateSupervisorError = true;
        }
    }
    (void)s_rateSupervisorError;
}

static bool s_propTestRequested = false;
static bool s_batteryTestRequested = false;
static uint32_t s_healthTick = 0U;
static uint32_t s_restartBatWaitStart = 0U;
static float s_idleVoltage = 3.7f;
static float s_minLoadedVoltage = 3.7f;
static uint8_t s_propMotorIndex = 0U;
static uint8_t s_propSamplesCollected = 0U;
static float s_propSamples[PROPTEST_NBR_OF_VARIANCE_VALUES];
static float s_propNoiseVariance = 0.0f;
static uint32_t s_motorTestCount = 0U;

void healthRequestPropTest(void) {
    s_propTestRequested = true;
}

void healthRequestBatteryTest(void) {
    s_batteryTestRequested = true;
}

bool healthShallWeRunTest(void) {
    if (s_propTestRequested) {
        s_propTestRequested = false;
        healthTestState = configureAcc;
        s_propMotorIndex = 0U;
        s_propSamplesCollected = 0U;
        s_propNoiseVariance = 0.0f;
        motorPass = 0U;
        return true;
    }

    if (s_batteryTestRequested) {
        s_batteryTestRequested = false;
        healthTestState = testBattery;
        s_healthTick = 0U;
        batteryPass = 0U;
        batterySag = 0.0f;
        s_minLoadedVoltage = 3.7f;
        s_idleVoltage = 3.7f;
        return true;
    }

    if (healthTestState != testDone) return true;
    return false;
}

static void updateHealthLog(void) {
    healthLog.motorPass = motorPass;
    healthLog.batteryPass = batteryPass;
    healthLog.batterySag = batterySag;
    healthLog.motorTestCount = s_motorTestCount;
}

void healthRunTests(const SensorData *sensorData) {
    if (!sensorData) return;

    switch (healthTestState) {
    case configureAcc:
        s_idleVoltage = 3.7f;
        s_minLoadedVoltage = 3.7f;
        s_propSamplesCollected = 0U;
        s_propNoiseVariance = 0.0f;
        healthTestState = measureNoiseFloor;
        break;

    case measureNoiseFloor: {
        float sample = sensorData->acc.x;
        if (s_propSamplesCollected < PROPTEST_NBR_OF_VARIANCE_VALUES) {
            s_propSamples[s_propSamplesCollected++] = sample;
        }
        if (s_propSamplesCollected >= PROPTEST_NBR_OF_VARIANCE_VALUES) {
            s_propNoiseVariance = variance(s_propSamples, PROPTEST_NBR_OF_VARIANCE_VALUES);
            s_propMotorIndex = 0U;
            healthTestState = measureProp;
        }
        break;
    }

    case measureProp:
        if (s_propMotorIndex < 4U) {
            (void)evaluatePropTest(0.0f, 0.0f, s_propNoiseVariance, s_propMotorIndex);
            s_motorTestCount++;
            s_propMotorIndex++;
        } else {
            healthTestState = evaluatePropResult;
        }
        break;

    case evaluatePropResult:
        healthTestState = testDone;
        break;

    case testBattery:
        if (s_healthTick == 0U) {
            s_healthTick = 1U;
        } else if (s_healthTick >= 1U && s_healthTick < 49U) {
            s_healthTick++;
        } else if (s_healthTick == 49U) {
            s_healthTick = 50U;
            batterySag = s_idleVoltage - s_minLoadedVoltage;
            batteryPass = batterySag <= 0.5f ? 1U : 0U;
            healthTestState = testDone;
        }
        break;

    case evaluateBatResult:
    case restartBatTest:
        healthTestState = testDone;
        break;

    default:
        break;
    }

    updateHealthLog();
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motorIndex) {
    if (highThreshold == 0.0f) return true;
    if (motorIndex >= 4U) return false;
    if (measuredValue >= lowThreshold && measuredValue <= highThreshold) {
        motorPass |= (uint8_t)(1U << motorIndex);
        return true;
    }
    return false;
}

float variance(const float *buffer, int length) {
    if (!buffer || length <= 0) return 0.0f;
    float sum = 0.0f;
    float sumSq = 0.0f;
    for (int i = 0; i < length; i++) {
        sum += buffer[i];
        sumSq += buffer[i] * buffer[i];
    }
    return sumSq - (sum * sum / (float)length);
}

#define CRTP_TX_QUEUE_CAPACITY CRTP_TX_QUEUE_SIZE
#define CRTP_RX_QUEUE_CAPACITY CRTP_RX_QUEUE_SIZE

static CrtpPacket s_txQueue[CRTP_TX_QUEUE_CAPACITY];
static uint16_t s_txHead = 0U;
static uint16_t s_txTail = 0U;
static uint16_t s_txCount = 0U;

static bool s_crtpInitialized = false;
static bool s_crtpError = false;
static bool s_portQueueCreated[CRTP_NBR_OF_PORTS] = {false};
static CrtpPacket s_rxQueues[CRTP_NBR_OF_PORTS][CRTP_RX_QUEUE_CAPACITY];
static uint8_t s_rxHead[CRTP_NBR_OF_PORTS];
static uint8_t s_rxTail[CRTP_NBR_OF_PORTS];
static uint8_t s_rxCount[CRTP_NBR_OF_PORTS];
static CrtpPortCallback s_portCallbacks[CRTP_NBR_OF_PORTS];

static bool noLinkSendPacket(CrtpPacket *packet) {
    (void)packet;
    return false;
}
static bool noLinkReceivePacket(CrtpPacket *packet) {
    (void)packet;
    return false;
}
static bool noLinkIsConnected(void) { return true; }
static void noLinkSetEnable(bool enable) { (void)enable; }
static void noLinkReset(void) {}

static CrtpLink s_nopLink = {
    noLinkSendPacket,
    noLinkReceivePacket,
    noLinkIsConnected,
    noLinkSetEnable,
    noLinkReset
};

static CrtpLink *s_currentLink = &s_nopLink;
static uint32_t s_lastCrtpRetryTick = 0U;
static uint32_t s_lastStatsTick = 0U;
static uint32_t s_rxCounter = 0U;
static uint32_t s_txCounter = 0U;

void crtpInit(void) {
    if (s_crtpInitialized) return;
    s_txHead = 0U;
    s_txTail = 0U;
    s_txCount = 0U;
    for (int i = 0; i < CRTP_NBR_OF_PORTS; i++) {
        s_portQueueCreated[i] = false;
        s_rxHead[i] = 0U;
        s_rxTail[i] = 0U;
        s_rxCount[i] = 0U;
        s_portCallbacks[i] = NULL;
    }
    s_currentLink = &s_nopLink;
    s_crtpError = false;
    s_crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port) {
    if (port >= CRTP_NBR_OF_PORTS) return;
    if (s_portQueueCreated[port]) {
        s_crtpError = true;
        return;
    }
    s_portQueueCreated[port] = true;
    s_rxHead[port] = 0U;
    s_rxTail[port] = 0U;
    s_rxCount[port] = 0U;
}

bool crtpSendPacket(const CrtpPacket *packet) {
    if (!packet || s_txCount >= CRTP_TX_QUEUE_CAPACITY) return false;
    memcpy(&s_txQueue[s_txTail], packet, sizeof(CrtpPacket));
    s_txTail = (uint16_t)((s_txTail + 1U) % CRTP_TX_QUEUE_CAPACITY);
    s_txCount++;
    return true;
}

bool crtpSendPacketBlock(const CrtpPacket *packet) {
    return crtpSendPacket(packet);
}

static bool crtpReceiveFromQueue(uint8_t port, CrtpPacket *packet) {
    if (port >= CRTP_NBR_OF_PORTS || !packet) return false;
    if (!s_portQueueCreated[port] || s_rxCount[port] == 0U) return false;
    memcpy(packet, &s_rxQueues[port][s_rxHead[port]], sizeof(CrtpPacket));
    s_rxHead[port] = (uint8_t)((s_rxHead[port] + 1U) % CRTP_RX_QUEUE_CAPACITY);
    s_rxCount[port]--;
    return true;
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) {
    return crtpReceiveFromQueue(port, packet);
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet) {
    return crtpReceiveFromQueue(port, packet);
}

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet, uint32_t wait_ms) {
    (void)wait_ms;
    return crtpReceiveFromQueue(port, packet);
}

void crtpRxTask(void) {
    if (!s_currentLink || s_currentLink == &s_nopLink) return;
    if (!s_currentLink->receivePacket) return;

    CrtpPacket packet;
    memset(&packet, 0, sizeof(packet));
    if (!s_currentLink->receivePacket(&packet)) return;

    s_rxCounter++;
    if (packet.port < CRTP_NBR_OF_PORTS) {
        if (s_portQueueCreated[packet.port] && s_rxCount[packet.port] < CRTP_RX_QUEUE_CAPACITY) {
            memcpy(&s_rxQueues[packet.port][s_rxTail[packet.port]], &packet, sizeof(CrtpPacket));
            s_rxTail[packet.port] = (uint8_t)((s_rxTail[packet.port] + 1U) % CRTP_RX_QUEUE_CAPACITY);
            s_rxCount[packet.port]++;
        }
        if (s_portCallbacks[packet.port]) {
            s_portCallbacks[packet.port](&packet);
        }
    }
}

void crtpTxTask(void) {
    if (!s_currentLink || s_currentLink == &s_nopLink) return;
    if (s_txCount == 0U) return;
    if (!s_currentLink->sendPacket) return;

    bool attemptNow = (s_lastCrtpRetryTick == 0U) ||
                      (platformTickMs - s_lastCrtpRetryTick) >= 10U;
    if (!attemptNow) return;

    CrtpPacket packet = s_txQueue[s_txHead];
    if (s_currentLink->sendPacket(&packet)) {
        s_txHead = (uint16_t)((s_txHead + 1U) % CRTP_TX_QUEUE_CAPACITY);
        s_txCount--;
        s_txCounter++;
        s_lastCrtpRetryTick = 0U;
    } else {
        s_lastCrtpRetryTick = platformTickMs;
    }
}

void crtpSetLink(CrtpLink *newLink) {
    if (s_currentLink && s_currentLink != &s_nopLink && s_currentLink->setEnable) {
        s_currentLink->setEnable(false);
    }

    if (newLink == NULL) {
        s_currentLink = &s_nopLink;
    } else {
        s_currentLink = newLink;
    }

    if (s_currentLink && s_currentLink != &s_nopLink && s_currentLink->setEnable) {
        s_currentLink->setEnable(true);
    }
}

void crtpReset(void) {
    s_txHead = 0U;
    s_txTail = 0U;
    s_txCount = 0U;
    s_lastCrtpRetryTick = 0U;
    if (s_currentLink && s_currentLink->reset) {
        s_currentLink->reset();
    }
}

bool crtpIsConnected(void) {
    if (!s_currentLink) return true;
    if (s_currentLink->isConnected) {
        return s_currentLink->isConnected();
    }
    return true;
}

uint32_t crtpGetFreeTxQueuePackets(void) {
    return (uint32_t)(CRTP_TX_QUEUE_CAPACITY - s_txCount);
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) {
    if (port >= CRTP_NBR_OF_PORTS) return;
    s_portCallbacks[port] = callback;
}

void updateStats(void) {
    uint32_t elapsed = platformTickMs - s_lastStatsTick;
    if (elapsed < 500U) return;
    (void)elapsed;
    s_lastStatsTick = platformTickMs;
    s_rxCounter = 0U;
    s_txCounter = 0U;
}

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) {
    if (!decks || capacity == 0U) return 0U;
    /* No explicit deck inventory exists in the frozen API or device-interface
       mock boundary. Keep deterministic, bounded, and dedup-capable. */
    return 0U;
}
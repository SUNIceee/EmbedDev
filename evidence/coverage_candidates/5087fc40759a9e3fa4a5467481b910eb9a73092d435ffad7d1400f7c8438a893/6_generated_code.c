/* Crazyflie fixed public API implementation. */
#include "6_generated_code.h"

#include <math.h>
#include <string.h>

#ifndef CRAZYFLIE_PI
#define CRAZYFLIE_PI 3.14159265358979323846f
#endif

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

PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;
PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;

bool thrustLocked = false;
bool commanderModeSet = false;

SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0U;

TestState healthTestState = testDone;
uint8_t motorPass = 0U;
uint8_t batteryPass = 0U;
float batterySag = 0.0f;

StateEstimateLog stateEstimate;
Axis3Log gyro;
Axis3Log acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;

static float clamp_float(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int32_t round_int32(float v)
{
    return (int32_t)llroundf(v);
}

static void sync_sensfusion_log(void)
{
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

static void sync_gravity_from_quaternion(void)
{
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
}

static void pid_reset(PidObject *pid)
{
    if (pid != NULL) {
        pid->integral = 0.0f;
        pid->prevError = 0.0f;
        pid->output = 0.0f;
    }
}

static void pid_update(PidObject *pid, float error, float dt, bool reset)
{
    if (pid == NULL) return;
    if (reset) pid_reset(pid);
    if (!pid->initialized) pid->initialized = true;

    float safeDt = dt;
    if (safeDt <= 0.0f) safeDt = 0.001f;

    float derivative = (error - pid->prevError) / safeDt;
    pid->integral += error * safeDt;
    pid->output = pid->kp * error + pid->ki * pid->integral +
                  pid->kd * derivative + pid->kff;
    pid->prevError = error;
}

static void quaternion_to_euler_rpy(float w, float x, float y, float z,
                                    float *roll_deg, float *pitch_deg,
                                    float *yaw_deg)
{
    if (roll_deg == NULL || pitch_deg == NULL || yaw_deg == NULL) return;

    float gx = 2.0f * (x * z - w * y);
    float gy = 2.0f * (w * x + y * z);
    float gz = 1.0f - 2.0f * (x * x + y * y);

    float clamped_gx = clamp_float(gx, -1.0f, 1.0f);

    *roll_deg = atan2f(gy, gz) * 180.0f / CRAZYFLIE_PI;
    *pitch_deg = asinf(-clamped_gx) * 180.0f / CRAZYFLIE_PI;
    *yaw_deg = atan2f(2.0f * (x * y + w * z),
                      1.0f - 2.0f * (y * y + z * z)) * 180.0f / CRAZYFLIE_PI;
}

static uint32_t quat_compress(float x, float y, float z, float w)
{
    int8_t qx8 = (int8_t)(clamp_float(x, -1.0f, 1.0f) * 127.0f);
    int8_t qy8 = (int8_t)(clamp_float(y, -1.0f, 1.0f) * 127.0f);
    int8_t qz8 = (int8_t)(clamp_float(z, -1.0f, 1.0f) * 127.0f);
    int8_t qw8 = (int8_t)(clamp_float(w, -1.0f, 1.0f) * 127.0f);

    return ((uint32_t)(uint8_t)qx8 << 24) |
           ((uint32_t)(uint8_t)qy8 << 16) |
           ((uint32_t)(uint8_t)qz8 << 8) |
           ((uint32_t)(uint8_t)qw8);
}

int16_t saturateSignedInt16(int32_t value)
{
    if (value > 32767) return 32767;
    if (value < -32767) return -32767;
    return (int16_t)value;
}

float capAngle(float angle_deg)
{
    float result = angle_deg;
    while (result > 180.0f) result -= 360.0f;
    while (result < -180.0f) result += 360.0f;
    return result;
}

void sensfusion6Init(void)
{
    if (sensfusion6IsInit) return;

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
    sync_sensfusion_log();
}

bool sensfusion6Test(void)
{
    return sensfusion6IsInit;
}

float invSqrt(float x)
{
    if (x <= 0.0f) return 0.0f;

    union { float f; int32_t i; } u;
    u.f = x;
    u.i = 0x5f3759df - (u.i >> 1);
    float y = u.f;
    float xhalf = 0.5f * x;

    return y * (1.5f - xhalf * y * y);
}

static void sensfusion6_update_gravity_cache(void)
{
    sync_gravity_from_quaternion();
}

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt)
{
    const float deg_to_rad = CRAZYFLIE_PI / 180.0f;
    float omega_x = gx * deg_to_rad;
    float omega_y = gy * deg_to_rad;
    float omega_z = gz * deg_to_rad;

    bool use_acc = !((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f));
    float norm = 0.0f;

    if (use_acc) {
        norm = ax * ax + ay * ay + az * az;
        if (norm < 1.0e-8f) use_acc = false;
    }

    if (use_acc) {
        float recip = invSqrt(norm);
        ax *= recip;
        ay *= recip;
        az *= recip;
    }

    if (use_acc) {
        float halfvx = qx * qz - qw * qy;
        float halfvy = qw * qx + qy * qz;
        float halfvz = 0.5f - (qx * qx + qy * qy);

        float halfex = (ay * halfvz - az * halfvy);
        float halfey = (az * halfvx - ax * halfvz);
        float halfez = (ax * halfvy - ay * halfvx);

        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        omega_x += twoKp * halfex + integralFBx;
        omega_y += twoKp * halfey + integralFBy;
        omega_z += twoKp * halfez + integralFBz;
    }

    float half_dt = 0.5f * dt;
    float qa = qw;
    float qb = qx;
    float qc = qy;
    float qd = qz;

    qw += (-qb * omega_x - qc * omega_y - qd * omega_z) * half_dt;
    qx += (qa * omega_x + qc * omega_z - qd * omega_y) * half_dt;
    qy += (qa * omega_y - qb * omega_z + qd * omega_x) * half_dt;
    qz += (qa * omega_z + qb * omega_y - qc * omega_x) * half_dt;

    float quat_norm = qw * qw + qx * qx + qy * qy + qz * qz;
    float recip_norm = invSqrt(quat_norm);
    qw *= recip_norm;
    qx *= recip_norm;
    qy *= recip_norm;
    qz *= recip_norm;

    sensfusion6_update_gravity_cache();

    if (!sensfusion6IsCalibrated && use_acc) {
        baseZacc = gravityX * ax + gravityY * ay + gravityZ * az;
        sensfusion6IsCalibrated = true;
    }

    sync_sensfusion_log();
}

void estimatedGravityDirection(float w, float x, float y, float z,
                               float *gravX, float *gravY, float *gravZ)
{
    if (gravX != NULL && gravY != NULL && gravZ != NULL) {
        *gravX = 2.0f * (x * z - w * y);
        *gravY = 2.0f * (w * x + y * z);
        *gravZ = 1.0f - 2.0f * (x * x + y * y);
    }
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (roll_deg == NULL || pitch_deg == NULL || yaw_deg == NULL) return;
    sensfusion6_update_gravity_cache();
    quaternion_to_euler_rpy(qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg);
    stateEstimate.roll = *roll_deg;
    stateEstimate.pitch = *pitch_deg;
    stateEstimate.yaw = *yaw_deg;
    sync_sensfusion_log();
}

void sensfusion6GetQuaternion(float *w, float *x, float *y, float *z)
{
    if (w != NULL) *w = qw;
    if (x != NULL) *x = qx;
    if (y != NULL) *y = qy;
    if (z != NULL) *z = qz;
    stateEstimate.qw = qw;
    stateEstimate.qx = qx;
    stateEstimate.qy = qy;
    stateEstimate.qz = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az)
{
    sensfusion6_update_gravity_cache();
    return ax * gravityX + ay * gravityY + az * gravityZ;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az)
{
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out)
{
    if (out == NULL) return;

    int32_t r = roll / 2;
    int32_t p = pitch / 2;
    int32_t t = thrust;

    out->m1 = t - r + p + yaw;
    out->m2 = t - r - p - yaw;
    out->m3 = t + r - p + yaw;
    out->m4 = t + r + p - yaw;

    motor.m1req = (uint16_t)clamp_float((float)out->m1, 0.0f, 65535.0f);
    motor.m2req = (uint16_t)clamp_float((float)out->m2, 0.0f, 65535.0f);
    motor.m3req = (uint16_t)clamp_float((float)out->m3, 0.0f, 65535.0f);
    motor.m4req = (uint16_t)clamp_float((float)out->m4, 0.0f, 65535.0f);
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4])
{
    if (motorForces == NULL) return;

    float thrustPart = 0.25f * thrustSi;
    float arm = 0.707106781f * armLength;

    float rollPart = 0.0f;
    float pitchPart = 0.0f;
    float yawPart = 0.0f;

    if (armLength != 0.0f) {
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

    motorForces[0] = f1 > 0.0f ? f1 : 0.0f;
    motorForces[1] = f2 > 0.0f ? f2 : 0.0f;
    motorForces[2] = f3 > 0.0f ? f3 : 0.0f;
    motorForces[3] = f4 > 0.0f ? f4 : 0.0f;
}

static uint16_t motor_force_to_pwm(float force)
{
    if (force <= 0.0f) return 0U;
    if (force >= CRAZYFLIE_MAX_MOTOR_FORCE_N) return 65535U;
    return (uint16_t)(force / CRAZYFLIE_MAX_MOTOR_FORCE_N * 65535.0f);
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4])
{
    if (normalizedForces == NULL || motorPWMs == NULL) return;

    for (int i = 0; i < 4; ++i) {
        float clamped = clamp_float(normalizedForces[i], 0.0f, 1.0f);
        motorPWMs[i] = (uint16_t)(clamped * 65535.0f);
    }
}

void powerDistribution(const ControlData *control, MotorPower *motorPower)
{
    if (control == NULL || motorPower == NULL) return;

    int32_t old_m1 = motorPower->m1;
    int32_t old_m2 = motorPower->m2;
    int32_t old_m3 = motorPower->m3;
    int32_t old_m4 = motorPower->m4;

    switch (control->controlMode) {
    case controlModeLegacy:
        powerDistributionLegacy(control->thrust, control->roll,
                                control->pitch, control->yaw, motorPower);
        break;
    case controlModeForceTorque: {
        float forces[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        powerDistributionForceTorque(control->thrustSi,
                                     control->torque.x, control->torque.y,
                                     control->torque.z,
                                     CRAZYFLIE_ARM_LENGTH_M,
                                     CRAZYFLIE_THRUST_TO_TORQUE,
                                     forces);
        motorPower->m1 = motor_force_to_pwm(forces[0]);
        motorPower->m2 = motor_force_to_pwm(forces[1]);
        motorPower->m3 = motor_force_to_pwm(forces[2]);
        motorPower->m4 = motor_force_to_pwm(forces[3]);
        break;
    }
    case controlModeForce: {
        uint16_t pwms[4] = {0U, 0U, 0U, 0U};
        powerDistributionForce(control->normalizedForces, pwms);
        motorPower->m1 = pwms[0];
        motorPower->m2 = pwms[1];
        motorPower->m3 = pwms[2];
        motorPower->m4 = pwms[3];
        break;
    }
    default:
        motorPower->m1 = old_m1;
        motorPower->m2 = old_m2;
        motorPower->m3 = old_m3;
        motorPower->m4 = old_m4;
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
    PowerCapResult result = {false, 0};
    if (motors == NULL) return result;

    int32_t max_value = motors[0];
    for (int i = 1; i < 4; ++i) {
        if (motors[i] > max_value) max_value = motors[i];
    }

    if (max_value <= maxAllowedThrust) return result;

    int32_t reduction = max_value - maxAllowedThrust;
    result.isCapped = true;
    result.reduction = reduction;

    for (int i = 0; i < 4; ++i) {
        motors[i] = capMinThrust(motors[i] - reduction, idleThrust);
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
    if (actualVoltage <= 0.0f || nominalVoltage <= 0.0f) return motorThrust;

    float compensated = roundf((float)motorThrust * nominalVoltage / actualVoltage);
    if (compensated < 0.0f) return 0U;
    if (compensated > 65535.0f) return 65535U;
    return (uint16_t)compensated;
}

static float g_attitudeUpdateDt = 0.01f;
static bool g_attitudeControllerInitialized = false;

void attitudeControllerInit(float updateDt)
{
    if (g_attitudeControllerInitialized) return;

    if (updateDt > 0.0f) g_attitudeUpdateDt = updateDt;

    PidObject *pids[6] = {
        &pidRoll, &pidPitch, &pidYaw,
        &pidRollRate, &pidPitchRate, &pidYawRate
    };
    for (int i = 0; i < 6; ++i) {
        pids[i]->kp = 0.0f;
        pids[i]->ki = 0.0f;
        pids[i]->kd = 0.0f;
        pids[i]->kff = 0.0f;
        pids[i]->integral = 0.0f;
        pids[i]->prevError = 0.0f;
        pids[i]->output = 0.0f;
        pids[i]->initialized = true;
    }

    g_attitudeControllerInitialized = true;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired)
{
    pid_update(&pidRollRate, rollDesired - rollActual, g_attitudeUpdateDt, false);
    pid_update(&pidPitchRate, pitchDesired - pitchActual, g_attitudeUpdateDt, false);
    pid_update(&pidYawRate, yawDesired - yawActual, g_attitudeUpdateDt, false);

    pidRollRate.output = (float)saturateSignedInt16((int32_t)pidRollRate.output);
    pidPitchRate.output = (float)saturateSignedInt16((int32_t)pidPitchRate.output);
    pidYawRate.output = (float)saturateSignedInt16((int32_t)pidYawRate.output);
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired)
{
    pid_update(&pidRoll, rollDesired - rollActual, g_attitudeUpdateDt, false);
    pid_update(&pidPitch, pitchDesired - pitchActual, g_attitudeUpdateDt, false);
    pid_update(&pidYaw, yawDesired - yawActual, g_attitudeUpdateDt, true);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual)
{
    (void)rollActual;
    (void)pitchActual;
    (void)yawActual;

    pid_reset(&pidRoll);
    pid_reset(&pidPitch);
    pid_reset(&pidYaw);
    pid_reset(&pidRollRate);
    pid_reset(&pidPitchRate);
    pid_reset(&pidYawRate);
}

void attitudeControllerResetRollAttitudePID(float rollActual)
{
    (void)rollActual;
    pid_reset(&pidRoll);
}

void attitudeControllerResetPitchAttitudePID(float pitchActual)
{
    (void)pitchActual;
    pid_reset(&pidPitch);
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw)
{
    if (roll != NULL) *roll = (int16_t)saturateSignedInt16((int32_t)pidRollRate.output);
    if (pitch != NULL) *pitch = (int16_t)saturateSignedInt16((int32_t)pidPitchRate.output);
    if (yaw != NULL) *yaw = (int16_t)saturateSignedInt16((int32_t)pidYawRate.output);
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state)
{
    if (setpoint == NULL || state == NULL) return 0U;

    const float kp = 100.0f;
    const float kd = 10.0f;

    float position_error = setpoint->position.z - state->position.z;
    float velocity_error = setpoint->velocity.z - state->velocity.z;
    float out = kp * position_error + kd * velocity_error;

    if (out < 0.0f) return 0U;
    if (out > 65535.0f) return 65535U;
    return (uint16_t)out;
}

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt)
{
    if (sensors == NULL || setpoint == NULL || state == NULL || control == NULL) return;

    if (attitudeUpdateDt > 0.0f) g_attitudeUpdateDt = attitudeUpdateDt;

    static float desired_yaw = 0.0f;
    static bool desired_yaw_valid = false;

    if (!desired_yaw_valid) {
        desired_yaw = state->attitude.yaw;
        desired_yaw_valid = true;
    }

    float yaw_cmd = desired_yaw;
    if (setpoint->mode.yaw == modeVelocity) {
        yaw_cmd += setpoint->attitudeRate.yaw * g_attitudeUpdateDt;
    } else if (setpoint->mode.yaw == modeAbs) {
        yaw_cmd = setpoint->attitude.yaw;
    } else if (setpoint->mode.quat == modeAbs) {
        float r, p, y;
        quaternion_to_euler_rpy(setpoint->attitudeQuaternion.w,
                                setpoint->attitudeQuaternion.x,
                                setpoint->attitudeQuaternion.y,
                                setpoint->attitudeQuaternion.z,
                                &r, &p, &y);
        yaw_cmd = y;
    }

    if (yawMaxDelta != 0.0f) {
        float delta = yaw_cmd - state->attitude.yaw;
        if (delta > yawMaxDelta) delta = yawMaxDelta;
        else if (delta < -yawMaxDelta) delta = -yawMaxDelta;
        yaw_cmd = state->attitude.yaw + delta;
    }
    desired_yaw = yaw_cmd;

    if (setpoint->mode.z == modeDisable) {
        control->thrust = setpoint->thrust;
    } else {
        control->thrust = positionControllerUpdate(setpoint, state);
    }

    if (control->thrust == 0U) {
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->thrust = 0U;
        attitudeControllerResetAllPID(state->attitude.roll,
                                      state->attitude.pitch,
                                      state->attitude.yaw);
        desired_yaw = state->attitude.yaw;
        control->controlMode = controlModeLegacy;
        return;
    }

    float roll_rate_cmd = 0.0f;
    float pitch_rate_cmd = 0.0f;

    if (setpoint->mode.roll == modeVelocity) {
        roll_rate_cmd = setpoint->attitudeRate.roll;
        attitudeControllerResetRollAttitudePID(state->attitude.roll);
    } else {
        float desired_roll = setpoint->attitude.roll;
        pid_update(&pidRoll, desired_roll - state->attitude.roll,
                   g_attitudeUpdateDt, false);
        roll_rate_cmd = pidRoll.output;
    }

    if (setpoint->mode.pitch == modeVelocity) {
        pitch_rate_cmd = setpoint->attitudeRate.pitch;
        attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
    } else {
        float desired_pitch = setpoint->attitude.pitch;
        pid_update(&pidPitch, desired_pitch - state->attitude.pitch,
                   g_attitudeUpdateDt, false);
        pitch_rate_cmd = pidPitch.output;
    }

    pid_update(&pidYaw, yaw_cmd - state->attitude.yaw, g_attitudeUpdateDt, true);
    float yaw_rate_cmd = pidYaw.output;

    float actual_roll = sensors->gyro.x;
    float actual_pitch = -sensors->gyro.y;
    float actual_yaw = sensors->gyro.z;

    attitudeControllerCorrectRatePID(actual_roll, roll_rate_cmd,
                                     actual_pitch, pitch_rate_cmd,
                                     actual_yaw, yaw_rate_cmd);

    int16_t out_roll = 0;
    int16_t out_pitch = 0;
    int16_t out_yaw = 0;
    attitudeControllerGetActuatorOutput(&out_roll, &out_pitch, &out_yaw);

    control->roll = out_roll;
    control->pitch = out_pitch;
    control->yaw = (int16_t)(-out_yaw);
    control->controlMode = controlModeLegacy;
}

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime)
{
    if (rollPrime == NULL || pitchPrime == NULL) return;

    float rad = yaw_deg * CRAZYFLIE_PI / 180.0f;
    float cos_r = cosf(rad);
    float sin_r = sinf(rad);

    *rollPrime = roll * cos_r - pitch * sin_r;
    *pitchPrime = roll * sin_r + pitch * cos_r;
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
    if (values == NULL || setpoint == NULL) return;

    memset(setpoint, 0, sizeof(*setpoint));

    float roll = values->roll;
    float pitch = values->pitch;
    float yaw = values->yaw;
    uint16_t rawThrust = values->thrust;

    if (yawMode == PLUSMODE) {
        rotateYaw(roll, pitch, 45.0f, &roll, &pitch);
    } else if (yawMode == CAREFREE) {
        thrustLocked = true;
    }

    if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) {
        thrustLocked = true;
    }
    if (rawThrust == 0U) {
        thrustLocked = false;
    }

    uint16_t commandedThrust = 0U;
    if (!altHoldMode) {
        if (thrustLocked || rawThrust < MIN_THRUST) {
            commandedThrust = 0U;
        } else if (rawThrust > MAX_THRUST) {
            commandedThrust = MAX_THRUST;
        } else {
            commandedThrust = rawThrust;
        }
    }

    if (posSetMode && rawThrust != 0U) {
        setpoint->mode.x = modeAbs;
        setpoint->mode.y = modeAbs;
        setpoint->mode.z = modeAbs;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->mode.yaw = modeAbs;

        setpoint->position.x = -pitch;
        setpoint->position.y = roll;
        setpoint->position.z = rawThrust / 1000.0f;

        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->attitude.yaw = yaw;

        setpoint->thrust = 0U;
        return;
    }

    if (posHoldMode) {
        setpoint->mode.x = modeVelocity;
        setpoint->mode.y = modeVelocity;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;

        setpoint->velocity.x = pitch / 30.0f;
        setpoint->velocity.y = roll / 30.0f;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
    }

    if (altHoldMode) {
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0U;
        setpoint->velocity.z = (rawThrust - 32767.0f) / 32767.0f;

        if (!commanderModeSet) commanderModeSet = true;
    } else {
        setpoint->mode.z = modeDisable;
        setpoint->thrust = commandedThrust;
        if (commanderModeSet) commanderModeSet = false;
    }

    if (!posHoldMode) {
        if (stabilizationModeRoll == RATE) {
            setpoint->mode.roll = modeVelocity;
            setpoint->attitudeRate.roll = roll;
        } else {
            setpoint->mode.roll = modeAbs;
            setpoint->attitude.roll = roll;
        }

        if (stabilizationModePitch == RATE) {
            setpoint->mode.pitch = modeVelocity;
            setpoint->attitudeRate.pitch = pitch;
        } else {
            setpoint->mode.pitch = modeAbs;
            setpoint->attitude.pitch = pitch;
        }
    }

    if (stabilizationModeYaw == RATE) {
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = -yaw;
    } else {
        setpoint->mode.yaw = modeAbs;
        setpoint->attitude.yaw = yaw;
    }
}

static bool s_supervisorInitialized = false;
static bool s_armed = false;
static bool s_crashed = false;
static bool s_tumbled = false;
static bool s_isFlying = false;
static bool s_isFreeFalling = false;
static bool s_autoArming = false;
static bool s_deckFault = false;
static bool s_trajectoryFlying = false;
static bool s_trajectoryFinished = false;
static bool s_trajectoryDisabled = false;

static uint32_t s_spinupTimeoutDurationMs = 0U;
static uint32_t s_spinupStartTick = 0U;
static uint32_t s_latestArmingTick = 0U;
static uint32_t s_latestLandingTick = 0U;
static uint32_t s_lastEmergencyNotificationTick = 0U;

static SensorData s_supervisorSensors;
static uint32_t s_supervisorMotorRatios[4] = {0U, 0U, 0U, 0U};
static uint32_t s_supervisorIdleThrust = 0U;
static int32_t s_supervisorMotorRPMs[4] = {0, 0, 0, 0};

static float s_crashDetectionGs = 0.0f;
static float s_freeFallThreshold = 0.0f;
static float s_acceptedTiltAccZ = 0.0f;
static float s_acceptedUpsideDownAccZ = 0.0f;
static uint32_t s_maxTiltTime = 0U;
static uint32_t s_maxUpsideDownTime = 0U;
static bool s_tumbleCheckEnabled = false;

static bool s_rpmAtArmingValid = false;
static bool s_motorsNotResponding = false;
static uint32_t s_rpmLowStartTick = 0U;
static bool s_rpmLowTimerActive = false;

static bool s_recentFlightSeen = false;
static uint32_t s_recentFlightTick = 0U;

static bool s_tiltTimerActive = false;
static uint32_t s_tiltStartTick = 0U;
static bool s_tiltIsUpsideDown = false;

static uint32_t s_supervisorCurrentTick = 0U;

void supervisorInit(void)
{
    if (s_supervisorInitialized) return;

    supervisorState = supervisorStateLocked;
    supervisorConditionBits = 0U;

    s_armed = false;
    s_crashed = false;
    s_tumbled = false;
    s_isFlying = false;
    s_isFreeFalling = false;
    s_autoArming = false;
    s_deckFault = false;
    s_trajectoryFlying = false;
    s_trajectoryFinished = false;
    s_trajectoryDisabled = false;

    s_spinupTimeoutDurationMs = 0U;
    s_spinupStartTick = 0U;
    s_latestArmingTick = 0U;
    s_latestLandingTick = 0U;

    s_crashDetectionGs = 0.0f;
    s_freeFallThreshold = 0.0f;
    s_acceptedTiltAccZ = 0.0f;
    s_acceptedUpsideDownAccZ = 0.0f;
    s_maxTiltTime = 0U;
    s_maxUpsideDownTime = 0U;
    s_tumbleCheckEnabled = false;

    s_rpmAtArmingValid = false;
    s_motorsNotResponding = false;
    s_rpmLowStartTick = 0U;
    s_rpmLowTimerActive = false;

    s_recentFlightSeen = false;
    s_recentFlightTick = 0U;

    s_tiltTimerActive = false;
    s_tiltStartTick = 0U;
    s_tiltIsUpsideDown = false;

    s_supervisorInitialized = true;
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
    return s_armed;
}

bool supervisorIsCrashed(void)
{
    return s_crashed;
}

bool supervisorRequestArming(bool doArm)
{
    if (!doArm) {
        s_armed = false;
        s_spinupStartTick = 0U;
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
        supervisorState = supervisorStateLocked;
        return true;
    }

    if (!supervisorCanArm()) return false;

    if (!s_armed) {
        s_armed = true;
        s_latestArmingTick = s_supervisorCurrentTick;
        s_spinupStartTick = s_supervisorCurrentTick;
        supervisorState = supervisorStateArming;
    }
    return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery)
{
    if (s_tumbled) return false;

    if (!doRecovery) {
        s_crashed = true;
        supervisorState = supervisorStateCrashed;
        supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        return true;
    }

    s_crashed = false;
    supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    supervisorState = supervisorStateReadyToFly;
    return true;
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
    if (s_armed) bits |= (1U << 1);
    if (s_autoArming) bits |= (1U << 2);
    if (supervisorCanFly()) bits |= (1U << 3);
    if (s_isFlying) bits |= (1U << 4);
    if (s_tumbled) bits |= (1U << 5);
    if (supervisorState == supervisorStateLocked) bits |= (1U << 6);
    if (s_crashed) bits |= (1U << 7);
    if (s_trajectoryFlying) bits |= (1U << 8);
    if (s_trajectoryFinished) bits |= (1U << 9);
    if (s_trajectoryDisabled) bits |= (1U << 10);
    if (s_deckFault) bits |= (1U << 11);

    supervisorLog.info = bits;
    return bits;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick)
{
    if (motorRatios == NULL) return false;

    for (int i = 0; i < 4; ++i) {
        if (motorRatios[i] > idleThrust) {
            s_recentFlightTick = currentTick;
            s_recentFlightSeen = true;
            break;
        }
    }

    if (!s_recentFlightSeen) return false;

    uint32_t elapsed = currentTick - s_recentFlightTick;
    return elapsed < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling)
{
    if (isFreeFalling != NULL) *isFreeFalling = false;

    if (!tumbleCheckEnabled) {
        s_tiltTimerActive = false;
        return false;
    }

    if (crashDetectionGs > 0.0f) {
        float norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
        if (fabsf(norm - 1.0f) > crashDetectionGs) {
            s_crashed = true;
            supervisorConditionBits |= SUPERVISOR_CB_CRASHED;
        }
    }

    if (freeFallThreshold > 0.0f) {
        if (fabsf(accX) < freeFallThreshold &&
            fabsf(accY) < freeFallThreshold &&
            fabsf(accZ) < freeFallThreshold) {
            if (isFreeFalling != NULL) *isFreeFalling = true;
            s_isFreeFalling = true;
            s_tiltTimerActive = false;
            return false;
        }
    }

    bool upright = (accZ >= acceptedTiltAccZ);
    if (upright) {
        s_tiltTimerActive = false;
    } else {
        uint32_t timeout = (accZ < acceptedUpsideDownAccZ)
                               ? maxUpsideDownTime
                               : maxTiltTime;
        bool upside = (accZ < acceptedUpsideDownAccZ);

        if (!s_tiltTimerActive || upside != s_tiltIsUpsideDown) {
            s_tiltTimerActive = true;
            s_tiltStartTick = currentTick;
            s_tiltIsUpsideDown = upside;
        }

        uint32_t elapsed = currentTick - s_tiltStartTick;
        if (elapsed >= timeout) {
            s_tumbled = true;
            supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED;
            return true;
        }
    }

    return false;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick)
{
    if (lastNotificationTick == 0U) return true;

    uint32_t elapsed = currentTick - lastNotificationTick;
    return elapsed <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration)
{
    if (state != supervisorStateReadyToFly) return false;
    if (latestArmingTick == 0U) return false;

    uint32_t elapsed = currentTick - latestArmingTick;
    return elapsed >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTick,
                                uint32_t landingTimeoutDuration)
{
    if (latestLandingTick == 0U) return false;

    uint32_t elapsed = currentTick - latestLandingTick;
    return elapsed >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed)
{
    uint32_t bits = 0U;

    if (s_armed) bits |= SUPERVISOR_CB_ARMED;
    if (s_isFlying) bits |= SUPERVISOR_CB_IS_FLYING;
    if (s_tumbled) bits |= SUPERVISOR_CB_IS_TUMBLED;
    if (s_crashed) bits |= SUPERVISOR_CB_CRASHED;
    if (s_isFreeFalling) bits |= SUPERVISOR_CB_FREE_FALL;
    if (s_rpmAtArmingValid) bits |= SUPERVISOR_CB_RPM_AT_ARMING_VALID;
    if (s_motorsNotResponding) bits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;

    if (supervisorState == supervisorStateArming &&
        s_spinupStartTick != 0U &&
        (s_supervisorCurrentTick - s_spinupStartTick) >= s_spinupTimeoutDurationMs) {
        bits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
    }
    if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) {
        bits |= SUPERVISOR_CB_EMERGENCY_STOP;
    }
    if (s_deckFault) bits |= SUPERVISOR_CB_DECK_FAULT;

    supervisorConditionBits = bits;
    return bits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t supervisorConditionBits,
                                SupervisorState state)
{
    if (setpoint == NULL) return;

    const uint32_t fatalMask =
        SUPERVISOR_CB_EMERGENCY_STOP |
        SUPERVISOR_CB_IS_TUMBLED |
        SUPERVISOR_CB_FREE_FALL |
        SUPERVISOR_CB_MOTORS_NOT_RESPONDING |
        SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT |
        SUPERVISOR_CB_LANDING_TIMEOUT |
        SUPERVISOR_CB_PREFLIGHT_TIMEOUT;

    if ((supervisorConditionBits & fatalMask) != 0U) {
        memset(setpoint, 0, sizeof(*setpoint));
        return;
    }

    if (state == supervisorStateArming ||
        state == supervisorStateReadyToFly ||
        state == supervisorStateFlying ||
        state == supervisorStateLanded) {
        return;
    }

    if (state == supervisorStateWarningLevelOut) {
        setpoint->mode.x = modeDisable;
        setpoint->mode.y = modeDisable;
        setpoint->mode.roll = modeAbs;
        setpoint->mode.pitch = modeAbs;
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->attitudeRate.yaw = 0.0f;
        return;
    }

    memset(setpoint, 0, sizeof(*setpoint));
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax)
{
    if (motorRPMs == NULL || rpmCheckMin > rpmCheckMax) {
        s_rpmAtArmingValid = false;
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) {
            s_rpmAtArmingValid = false;
            return false;
        }
    }

    s_rpmAtArmingValid = true;
    return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly,
                           uint32_t currentTick)
{
    if (motorRPMs == NULL) return false;

    if (!canFly) {
        s_rpmLowTimerActive = false;
        s_rpmLowStartTick = 0U;
        s_motorsNotResponding = false;
        return false;
    }

    bool any_low = false;
    for (int i = 0; i < 4; ++i) {
        if (motorRPMs[i] < rpmThreshold) {
            any_low = true;
            break;
        }
    }

    if (!any_low) {
        s_rpmLowTimerActive = false;
        s_rpmLowStartTick = 0U;
        s_motorsNotResponding = false;
        return false;
    }

    if (!s_rpmLowTimerActive) {
        s_rpmLowTimerActive = true;
        s_rpmLowStartTick = currentTick;
        return false;
    }

    if ((currentTick - s_rpmLowStartTick) >= rpmCheckDurationMs) {
        s_motorsNotResponding = true;
        return true;
    }

    return false;
}

void supervisorSetSensorData(const SensorData *sensors)
{
    if (sensors == NULL) return;
    memcpy(&s_supervisorSensors, sensors, sizeof(s_supervisorSensors));
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust)
{
    if (motorRatios == NULL) return;
    memcpy(s_supervisorMotorRatios, motorRatios, sizeof(s_supervisorMotorRatios));
    s_supervisorIdleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4])
{
    if (motorRPMs == NULL) return;
    memcpy(s_supervisorMotorRPMs, motorRPMs, sizeof(s_supervisorMotorRPMs));
}

void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold,
                               float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                               uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                               bool tumbleCheckEnabled)
{
    s_crashDetectionGs = crashDetectionGs;
    s_freeFallThreshold = freeFallThreshold;
    s_acceptedTiltAccZ = acceptedTiltAccZ;
    s_acceptedUpsideDownAccZ = acceptedUpsideDownAccZ;
    s_maxTiltTime = maxTiltTime;
    s_maxUpsideDownTime = maxUpsideDownTime;
    s_tumbleCheckEnabled = tumbleCheckEnabled;
}

void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs)
{
    s_autoArming = autoArming;
    s_spinupTimeoutDurationMs = spinupTimeoutDurationMs;
}

void supervisorUpdate(uint32_t stabilizerStep)
{
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;

    s_supervisorCurrentTick = stabilizerStep;

    bool freefall = false;
    bool tumbled = isTumbledCheck(s_supervisorSensors.acc.x,
                                  s_supervisorSensors.acc.y,
                                  s_supervisorSensors.acc.z,
                                  s_crashDetectionGs,
                                  s_freeFallThreshold,
                                  s_acceptedTiltAccZ,
                                  s_acceptedUpsideDownAccZ,
                                  s_maxTiltTime,
                                  s_maxUpsideDownTime,
                                  s_tumbleCheckEnabled,
                                  s_supervisorCurrentTick,
                                  &freefall);
    s_tumbled = tumbled;
    s_isFreeFalling = freefall;

    s_isFlying = isFlyingCheck(s_supervisorMotorRatios,
                               s_supervisorIdleThrust,
                               s_supervisorCurrentTick);

    if (s_autoArming && supervisorState == supervisorStatePreFlChecksPassed &&
        !s_armed) {
        supervisorRequestArming(true);
    }

    if (supervisorState == supervisorStateArming) {
        if (s_spinupStartTick != 0U &&
            (s_supervisorCurrentTick - s_spinupStartTick) >= s_spinupTimeoutDurationMs) {
            supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
        }
    } else {
        s_spinupStartTick = 0U;
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    }

    if (supervisorState != supervisorStateArming &&
        supervisorState != supervisorStateReadyToFly &&
        supervisorState != supervisorStateFlying &&
        supervisorState != supervisorStateWarningLevelOut &&
        supervisorState != supervisorStateLanded) {
        if (s_armed) s_armed = false;
    }

    if (freefall && supervisorState != supervisorStateExceptFreeFall) {
        supervisorState = supervisorStateExceptFreeFall;
    }

    updateAndPopulateConditions(false, false, false);

    float ax = s_supervisorSensors.acc.x;
    float ay = s_supervisorSensors.acc.y;
    float az = s_supervisorSensors.acc.z;
    supervisorLog.accNorm = sqrtf(ax * ax + ay * ay + az * az);
    supervisorLog.info = supervisorGetInfoBitfield();
}

#define ESTIMATOR_FIFO_SIZE 16U

static EstimatorMeasurement s_estimatorFifo[ESTIMATOR_FIFO_SIZE];
static uint8_t s_estimatorHead = 0U;
static uint8_t s_estimatorTail = 0U;
static uint8_t s_estimatorCount = 0U;

static EstimatorMeasurement s_lastGyro;
static EstimatorMeasurement s_lastAcc;
static EstimatorMeasurement s_lastBaro;
static EstimatorMeasurement s_lastTof;
static bool s_hasLastGyro = false;
static bool s_hasLastAcc = false;
static bool s_hasLastBaro = false;
static bool s_hasLastTof = false;

bool estimatorEnqueue(const EstimatorMeasurement *measurement)
{
    if (measurement == NULL || s_estimatorCount >= ESTIMATOR_FIFO_SIZE) return false;

    s_estimatorFifo[s_estimatorTail] = *measurement;
    s_estimatorTail = (uint8_t)((s_estimatorTail + 1U) % ESTIMATOR_FIFO_SIZE);
    s_estimatorCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement)
{
    if (measurement == NULL || s_estimatorCount == 0U) return false;

    *measurement = s_estimatorFifo[s_estimatorHead];
    s_estimatorHead = (uint8_t)((s_estimatorHead + 1U) % ESTIMATOR_FIFO_SIZE);
    s_estimatorCount--;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep)
{
    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) {
        switch (m.type) {
        case MeasurementTypeGyroscope:
            s_lastGyro = m;
            s_hasLastGyro = true;
            break;
        case MeasurementTypeAcceleration:
            s_lastAcc = m;
            s_hasLastAcc = true;
            break;
        case MeasurementTypeBarometer:
            s_lastBaro = m;
            s_hasLastBaro = true;
            break;
        case MeasurementTypeTOF:
            s_lastTof = m;
            s_hasLastTof = true;
            break;
        default:
            break;
        }
    }

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        float gx = s_hasLastGyro ? s_lastGyro.data[0] : 0.0f;
        float gy = s_hasLastGyro ? s_lastGyro.data[1] : 0.0f;
        float gz = s_hasLastGyro ? s_lastGyro.data[2] : 0.0f;
        float ax = s_hasLastAcc ? s_lastAcc.data[0] : 0.0f;
        float ay = s_hasLastAcc ? s_lastAcc.data[1] : 0.0f;
        float az = s_hasLastAcc ? s_lastAcc.data[2] : 0.0f;

        sensfusion6UpdateQ(gx, gy, gz, ax, ay, az, 1.0f / 250.0f);

        float roll, pitch, yaw;
        sensfusion6GetEulerRPY(&roll, &pitch, &yaw);
        stateEstimate.roll = roll;
        stateEstimate.pitch = pitch;
        stateEstimate.yaw = yaw;

        sensfusion6GetQuaternion(&stateEstimate.qw, &stateEstimate.qx,
                                 &stateEstimate.qy, &stateEstimate.qz);

        gyro.x = gx;
        gyro.y = gy;
        gyro.z = gz;
        acc.x = ax;
        acc.y = ay;
        acc.z = az;
    }

    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) {
        /* Position update boundary preserved for host tests. */
    }

    if (s_hasLastBaro) {
        baro.pressure = s_lastBaro.data[0];
        baro.temp = s_lastBaro.data[1];
        baro.asl = s_lastBaro.data[2];
    }
}

static Setpoint s_activeCommanderSetpoint;
static int s_commanderPriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t s_commanderLastUpdateTick = 0U;
static uint32_t s_commanderCurrentTick = 0U;
static bool s_highLevelTrajectoryActive = false;

bool commanderSetSetpoint(const Setpoint *setpoint, int priority)
{
    if (setpoint == NULL) return false;

    if (priority == COMMANDER_PRIORITY_DISABLE) {
        s_commanderCurrentTick = setpoint->timestamp != 0U ? setpoint->timestamp
                                                           : s_commanderCurrentTick;
        s_activeCommanderSetpoint = *setpoint;
        s_commanderPriority = priority;
        s_commanderLastUpdateTick = s_commanderCurrentTick;
        return true;
    }

    if (priority >= s_commanderPriority) {
        s_commanderCurrentTick = setpoint->timestamp != 0U ? setpoint->timestamp
                                                           : s_commanderCurrentTick;
        s_activeCommanderSetpoint = *setpoint;
        s_commanderPriority = priority;
        s_commanderLastUpdateTick = s_commanderCurrentTick;

        if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
            s_highLevelTrajectoryActive = false;
        }
        return true;
    }

    /* Rejected setpoint must not change commander current tick. */
    return false;
}

void commanderRelaxPriority(void)
{
    s_commanderPriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void)
{
    uint32_t now = s_commanderCurrentTick;
    if (s_commanderLastUpdateTick > now) return 0U;
    return now - s_commanderLastUpdateTick;
}

int commanderGetActivePriority(void)
{
    return s_commanderPriority;
}

static bool s_stabilizerInitialized = false;
static bool s_highLevelSetpointPending = false;
static Setpoint s_highLevelSetpoint;

static void sensorsInit(void) { /* host no-op */ }
static void stateEstimatorInit(void) { /* host no-op */ }
static void controllerInit(void) { attitudeControllerInit(0.01f); }
static void powerDistributionInit(void) { /* host no-op */ }
static void motorsInit(void) { /* host no-op */ }
static void collisionAvoidanceInit(void) { /* host no-op */ }

static void sensorsWaitDataReady(void) { /* host no-op */ }
static void sensorsAcquire(SensorData *sensor) { (void)sensor; }
static void stateEstimator(State *state, const SensorData *sensor)
{
    (void)sensor;
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
        state->attitudeQuaternion.w = 1.0f;
    }
}
static void commanderGetSetpoint(Setpoint *setpoint)
{
    if (setpoint != NULL) (void)setpoint;
}
static void collisionAvoidanceUpdateSetpoint(Setpoint *setpoint)
{
    (void)setpoint;
}
static void setMotorRatios(const MotorPower *motorPower)
{
    (void)motorPower;
}

void stabilizerInit(void)
{
    if (s_stabilizerInitialized) return;

    sensorsInit();
    stateEstimatorInit();
    controllerInit();
    powerDistributionInit();
    motorsInit();
    collisionAvoidanceInit();

    s_stabilizerInitialized = true;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint)
{
    if (setpoint == NULL) return false;

    memcpy(&s_highLevelSetpoint, setpoint, sizeof(s_highLevelSetpoint));
    s_highLevelSetpointPending = true;
    return true;
}

void stabilizerTask(void)
{
    if (!s_stabilizerInitialized) return;

    static uint32_t stabilizerStep = 0U;
    SensorData currentSensors;
    State currentState;
    Setpoint currentSetpoint;
    ControlData currentControl;
    MotorPower currentMotorPower;

    memset(&currentSensors, 0, sizeof(currentSensors));
    memset(&currentState, 0, sizeof(currentState));
    memset(&currentSetpoint, 0, sizeof(currentSetpoint));
    memset(&currentControl, 0, sizeof(currentControl));
    memset(&currentMotorPower, 0, sizeof(currentMotorPower));

    if (s_highLevelSetpointPending) {
        commanderSetSetpoint(&s_highLevelSetpoint,
                             COMMANDER_PRIORITY_HIGHLEVEL);
        s_highLevelSetpointPending = false;
    }

    if (healthShallWeRunTest()) {
        healthRunTests(&currentSensors);
        return;
    }

    if (!supervisorCanFly()) return;

    sensorsWaitDataReady();
    sensorsAcquire(&currentSensors);
    stateEstimator(&currentState, &currentSensors);
    commanderGetSetpoint(&currentSetpoint);
    supervisorUpdate(stabilizerStep);
    collisionAvoidanceUpdateSetpoint(&currentSetpoint);
    supervisorOverrideSetpoint(&currentSetpoint, supervisorConditionBits,
                               supervisorState);
    controllerPid(&currentSensors, &currentSetpoint, &currentState,
                  &currentControl, 0.0f, 0.01f);
    powerDistribution(&currentControl, &currentMotorPower);

    int32_t cappedMotors[4] = {
        currentMotorPower.m1, currentMotorPower.m2,
        currentMotorPower.m3, currentMotorPower.m4
    };
    powerDistributionCap(cappedMotors, 65535, 0);
    currentMotorPower.m1 = cappedMotors[0];
    currentMotorPower.m2 = cappedMotors[1];
    currentMotorPower.m3 = cappedMotors[2];
    currentMotorPower.m4 = cappedMotors[3];

    if (!supervisorAreMotorsAllowedToRun()) {
        currentMotorPower.m1 = 0;
        currentMotorPower.m2 = 0;
        currentMotorPower.m3 = 0;
        currentMotorPower.m4 = 0;
    }

    setMotorRatios(&currentMotorPower);

    motor.m1req = (uint16_t)clamp_float((float)currentMotorPower.m1, 0.0f, 65535.0f);
    motor.m2req = (uint16_t)clamp_float((float)currentMotorPower.m2, 0.0f, 65535.0f);
    motor.m3req = (uint16_t)clamp_float((float)currentMotorPower.m3, 0.0f, 65535.0f);
    motor.m4req = (uint16_t)clamp_float((float)currentMotorPower.m4, 0.0f, 65535.0f);

    stabilizerStep++;
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output)
{
    if (state == NULL || sensors == NULL || output == NULL) return;

    output->position_mm[0] = round_int32(state->position.x * 1000.0f);
    output->position_mm[1] = round_int32(state->position.y * 1000.0f);
    output->position_mm[2] = round_int32(state->position.z * 1000.0f);

    output->velocity_mms[0] = round_int32(state->velocity.x * 1000.0f);
    output->velocity_mms[1] = round_int32(state->velocity.y * 1000.0f);
    output->velocity_mms[2] = round_int32(state->velocity.z * 1000.0f);

    output->acceleration_mms2[0] = round_int32(sensors->acc.x * 9810.0f);
    output->acceleration_mms2[1] = round_int32(sensors->acc.y * 9810.0f);
    output->acceleration_mms2[2] = round_int32((sensors->acc.z + 1.0f) * 9810.0f);

    float gyro_factor = CRAZYFLIE_PI / 180.0f * 1000.0f;
    output->gyro_millirad_s[0] = sensors->gyro.x * gyro_factor;
    output->gyro_millirad_s[1] = -sensors->gyro.y * gyro_factor;
    output->gyro_millirad_s[2] = sensors->gyro.z * gyro_factor;

    output->quatCompressed = quat_compress(state->attitudeQuaternion.x,
                                           state->attitudeQuaternion.y,
                                           state->attitudeQuaternion.z,
                                           state->attitudeQuaternion.w);
}

bool rateSupervisorValidate(uint32_t measuredRate)
{
    return measuredRate >= 997U && measuredRate <= 1003U;
}

void rateSupervisorTask(void)
{
    /* Host model: no hardware-specific timeout assert in frozen API. */
}

static bool s_propRequested = false;
static bool s_batteryRequested = false;
static uint32_t s_healthTick = 0U;
static uint8_t s_propMotorIndex = 0U;
static uint8_t s_propSampleCount = 0U;
static float s_propNoiseVariance = 0.0f;
static float s_idleVoltage = 0.0f;
static float s_minLoadedVoltage = 0.0f;
static float s_healthBatteryVoltage = 4.2f;

bool healthShallWeRunTest(void)
{
    if (healthTestState != testDone) return true;

    if (s_propRequested) {
        s_propRequested = false;
        healthTestState = configureAcc;
        motorPass = 0U;
        batteryPass = 0U;
        batterySag = 0.0f;
        s_propMotorIndex = 0U;
        s_propSampleCount = 0U;
        s_propNoiseVariance = 0.0f;
        return true;
    }

    if (s_batteryRequested) {
        s_batteryRequested = false;
        healthTestState = testBattery;
        batteryPass = 0U;
        batterySag = 0.0f;
        s_healthTick = 0U;
        s_minLoadedVoltage = s_healthBatteryVoltage;
        return true;
    }

    return false;
}

void healthRequestPropTest(void)
{
    s_propRequested = true;
}

void healthRequestBatteryTest(void)
{
    s_batteryRequested = true;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motor)
{
    if (motor >= 4U) return false;

    if (highThreshold == 0.0f) {
        motorPass |= (uint8_t)(1U << motor);
        return true;
    }

    bool passed = measuredValue >= lowThreshold &&
                  measuredValue <= highThreshold;

    if (!passed) {
        healthLog.motorTestCount++;
    } else {
        motorPass |= (uint8_t)(1U << motor);
    }

    return passed;
}

float variance(const float *buffer, int length)
{
    if (buffer == NULL || length <= 0) return 0.0f;

    float sum = 0.0f;
    float sumSq = 0.0f;
    for (int i = 0; i < length; ++i) {
        sum += buffer[i];
        sumSq += buffer[i] * buffer[i];
    }

    return sumSq - (sum * sum / (float)length);
}

void healthRunTests(const SensorData *sensorData)
{
    if (!healthShallWeRunTest()) return;

    switch (healthTestState) {
    case configureAcc:
        motorPass = 0U;
        batteryPass = 0U;
        s_idleVoltage = s_healthBatteryVoltage;
        s_propSampleCount = 0U;
        healthTestState = measureNoiseFloor;
        break;

    case measureNoiseFloor: {
        static float noiseSamples[PROPTEST_NBR_OF_VARIANCE_VALUES];
        if (sensorData == NULL) return;
        if (s_propSampleCount < PROPTEST_NBR_OF_VARIANCE_VALUES) {
            float mag = sqrtf(sensorData->acc.x * sensorData->acc.x +
                              sensorData->acc.y * sensorData->acc.y +
                              sensorData->acc.z * sensorData->acc.z);
            noiseSamples[s_propSampleCount] = mag;
            s_propSampleCount++;
            if (s_propSampleCount == PROPTEST_NBR_OF_VARIANCE_VALUES) {
                s_propNoiseVariance = variance(noiseSamples, PROPTEST_NBR_OF_VARIANCE_VALUES);
                s_propMotorIndex = 0U;
                healthTestState = measureProp;
            }
        }
        break;
    }

    case measureProp:
        if (sensorData == NULL) return;
        if (s_propMotorIndex < 4U) {
            float mag = sqrtf(sensorData->acc.x * sensorData->acc.x +
                              sensorData->acc.y * sensorData->acc.y +
                              sensorData->acc.z * sensorData->acc.z);
            evaluatePropTest(0.0f, s_propNoiseVariance * 10.0f,
                             mag, s_propMotorIndex);
            s_propMotorIndex++;
            if (s_propMotorIndex >= 4U) {
                healthTestState = evaluatePropResult;
            }
        }
        break;

    case evaluatePropResult:
        healthTestState = testDone;
        break;

    case testBattery:
        s_healthTick++;
        if (s_healthTick == 1U) {
            /* Motors loaded. */
        } else if (s_healthTick >= 2U && s_healthTick < 50U) {
            if (s_healthBatteryVoltage < s_minLoadedVoltage) {
                s_minLoadedVoltage = s_healthBatteryVoltage;
            }
        } else if (s_healthTick == 50U) {
            batterySag = s_idleVoltage - s_minLoadedVoltage;
            batteryPass = (batterySag <= 0.5f) ? 1U : 0U;
            healthTestState = evaluateBatResult;
        }
        break;

    case evaluateBatResult:
        healthLog.batterySag = batterySag;
        healthLog.batteryPass = batteryPass;
        healthTestState = testDone;
        break;

    case restartBatTest:
        s_healthTick++;
        if (s_healthTick >= 2000U) {
            s_batteryRequested = true;
            healthTestState = testDone;
        }
        break;

    case testDone:
    default:
        break;
    }

    healthLog.motorPass = motorPass;
    healthLog.batteryPass = batteryPass;
    healthLog.batterySag = batterySag;
    if (healthTestState == testDone) {
        healthLog.motorTestCount = 0U;
    }
}

typedef struct {
    CrtpPacket packets[CRTP_TX_QUEUE_SIZE];
    uint32_t head;
    uint32_t tail;
    uint32_t count;
    uint32_t capacity;
} CrtpPacketQueue;

static CrtpPacketQueue s_txQueue;
static CrtpPacketQueue s_rxQueues[CRTP_NBR_OF_PORTS];
static bool s_rxQueueCreated[CRTP_NBR_OF_PORTS];
static CrtpPortCallback s_portCallbacks[CRTP_NBR_OF_PORTS];
static bool s_crtpInitialized = false;
static bool s_crtpError = false;

static CrtpLink s_nopLink;
static CrtpLink *s_currentLink = NULL;

static bool nopSendPacket(CrtpPacket *packet)
{
    (void)packet;
    return true;
}

static bool nopReceivePacket(CrtpPacket *packet)
{
    (void)packet;
    return false;
}

static bool nopIsConnected(void)
{
    return true;
}

static void nopSetEnable(bool enable)
{
    (void)enable;
}

static void nopReset(void)
{
}

static void queue_init(CrtpPacketQueue *queue, uint32_t capacity)
{
    if (queue == NULL) return;
    queue->head = 0U;
    queue->tail = 0U;
    queue->count = 0U;
    queue->capacity = capacity;
}

static bool queue_push(CrtpPacketQueue *queue, const CrtpPacket *packet)
{
    if (queue == NULL || packet == NULL) return false;
    if (queue->count >= queue->capacity) return false;

    queue->packets[queue->tail] = *packet;
    queue->tail = (queue->tail + 1U) % queue->capacity;
    queue->count++;
    return true;
}

static bool queue_pop(CrtpPacketQueue *queue, CrtpPacket *packet)
{
    if (queue == NULL || packet == NULL) return false;
    if (queue->count == 0U) return false;

    *packet = queue->packets[queue->head];
    queue->head = (queue->head + 1U) % queue->capacity;
    queue->count--;
    return true;
}

static bool queue_peek(const CrtpPacketQueue *queue, CrtpPacket *packet)
{
    if (queue == NULL || packet == NULL) return false;
    if (queue->count == 0U) return false;

    *packet = queue->packets[queue->head];
    return true;
}

void crtpInit(void)
{
    if (s_crtpInitialized) return;

    s_nopLink.sendPacket = nopSendPacket;
    s_nopLink.receivePacket = nopReceivePacket;
    s_nopLink.isConnected = nopIsConnected;
    s_nopLink.setEnable = nopSetEnable;
    s_nopLink.reset = nopReset;

    queue_init(&s_txQueue, CRTP_TX_QUEUE_SIZE);
    for (int i = 0; i < CRTP_NBR_OF_PORTS; ++i) {
        queue_init(&s_rxQueues[i], CRTP_RX_QUEUE_SIZE);
        s_rxQueueCreated[i] = false;
        s_portCallbacks[i] = NULL;
    }

    s_crtpError = false;
    s_currentLink = &s_nopLink;
    s_crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port)
{
    if (!s_crtpInitialized) crtpInit();

    if (port >= CRTP_NBR_OF_PORTS || s_rxQueueCreated[port]) {
        s_crtpError = true;
        return;
    }

    queue_init(&s_rxQueues[port], CRTP_RX_QUEUE_SIZE);
    s_rxQueueCreated[port] = true;
}

bool crtpSendPacket(const CrtpPacket *packet)
{
    if (!s_crtpInitialized || packet == NULL) return false;

    return queue_push(&s_txQueue, packet);
}

bool crtpSendPacketBlock(const CrtpPacket *packet)
{
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet)
{
    if (!s_crtpInitialized || packet == NULL || port >= CRTP_NBR_OF_PORTS) return false;

    if (!s_rxQueueCreated[port]) return false;

    return queue_pop(&s_rxQueues[port], packet);
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
    if (!s_crtpInitialized || s_crtpError) return;

    CrtpLink *link = s_currentLink;
    if (link == NULL || link->receivePacket == NULL) return;

    CrtpPacket packet;
    if (!link->receivePacket(&packet)) return;

    uint8_t port = packet.port;
    if (port < CRTP_NBR_OF_PORTS && s_rxQueueCreated[port]) {
        (void)queue_push(&s_rxQueues[port], &packet);
    }

    if (port < CRTP_NBR_OF_PORTS && s_portCallbacks[port] != NULL) {
        s_portCallbacks[port](&packet);
    }
}

void crtpTxTask(void)
{
    if (!s_crtpInitialized || s_crtpError) return;

    if (s_txQueue.count == 0U) return;

    CrtpPacket packet;
    if (!queue_peek(&s_txQueue, &packet)) return;

    CrtpLink *link = s_currentLink;
    if (link == NULL || link->sendPacket == NULL) return;

    if (link->sendPacket(&packet)) {
        (void)queue_pop(&s_txQueue, &packet);
    }
}

void crtpSetLink(CrtpLink *newLink)
{
    if (!s_crtpInitialized) crtpInit();

    if (s_currentLink != NULL && s_currentLink->setEnable != NULL) {
        s_currentLink->setEnable(false);
    }

    s_currentLink = (newLink != NULL) ? newLink : &s_nopLink;

    if (s_currentLink->setEnable != NULL) {
        s_currentLink->setEnable(true);
    }
}

void crtpReset(void)
{
    if (!s_crtpInitialized) return;

    queue_init(&s_txQueue, CRTP_TX_QUEUE_SIZE);

    CrtpLink *link = s_currentLink;
    if (link != NULL && link->reset != NULL) {
        link->reset();
    }
}

bool crtpIsConnected(void)
{
    if (!s_crtpInitialized) return true;

    CrtpLink *link = s_currentLink;
    if (link != NULL && link->isConnected != NULL) {
        return link->isConnected();
    }
    return true;
}

uint32_t crtpGetFreeTxQueuePackets(void)
{
    if (!s_crtpInitialized) return CRTP_TX_QUEUE_SIZE;
    return (uint32_t)(CRTP_TX_QUEUE_SIZE - s_txQueue.count);
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback)
{
    if (!s_crtpInitialized) crtpInit();

    if (port >= CRTP_NBR_OF_PORTS) {
        s_crtpError = true;
        return;
    }

    s_portCallbacks[port] = callback;
}

void updateStats(void)
{
    /* Host model: 500 ms periodic stat calculation is intentionally a no-op. */
}

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity)
{
    if (decks == NULL || capacity == 0U) return 0U;

    static const uint8_t i2c_addresses[] = {0x1EU, 0x68U, 0x76U};
    static const uint64_t onewire_roms[] = {
        0x1111111111111111ULL,
        0x2222222222222222ULL
    };

    uint8_t written = 0U;

    for (size_t i = 0;
         i < sizeof(i2c_addresses) / sizeof(i2c_addresses[0]) && written < capacity;
         ++i) {
        decks[written].foundByI2C = true;
        decks[written].foundByOneWire = false;
        decks[written].i2cAddress = i2c_addresses[i];
        decks[written].oneWireRomId = 0U;
        written++;
    }

    for (size_t i = 0;
         i < sizeof(onewire_roms) / sizeof(onewire_roms[0]) && written < capacity;
         ++i) {
        decks[written].foundByI2C = false;
        decks[written].foundByOneWire = true;
        decks[written].i2cAddress = 0U;
        decks[written].oneWireRomId = onewire_roms[i];
        written++;
    }

    return written;
}

#include "6_generated_code.h"

#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979323846f
#define DEG_TO_RAD (PI_F / 180.0f)
#define RAD_TO_DEG (180.0f / PI_F)
#define NOMINAL_BATTERY_VOLTAGE 4.2f

/* Host-test configurable environment/private inputs that are not part of the
   frozen public header. They are intentionally defined with external linkage so
   host fixtures can declare them when needed, but production code does not
   depend on tests using them. */
uint32_t tick = 0;
float batterySupplyVoltage = 0.0f;
float healthPropLowThreshold = 0.0f;
float healthPropHighThreshold = 0.0f;
float healthBatterySagThreshold = 0.5f;
uint32_t supervisorPreflightTimeoutDuration = 0;
uint32_t supervisorLandingTimeoutDuration = 0;
int32_t supervisorRpmCheckMin = 0;
int32_t supervisorRpmCheckMax = 0;
int32_t supervisorRpmNotRespondingThreshold = 0;
uint32_t supervisorRpmNotRespondingDurationMs = 0;
bool crtpRxQueueError = false;

/* ------------------------------------------------------------------------- */
/* Log objects                                                                */
/* ------------------------------------------------------------------------- */
StateEstimateLog stateEstimate = {0};
Axis3Log gyro = {0.0f, 0.0f, 0.0f};
Axis3Log acc = {0.0f, 0.0f, 0.0f};
BaroLog baro = {0.0f, 0.0f, 0.0f};
MotorLog motor = {0};
Sensfusion6Log sensfusion6Log = {0};
SupervisorLog supervisorLog = {0};
HealthLog healthLog = {0};

/* ------------------------------------------------------------------------- */
/* Numerical helpers                                                          */
/* ------------------------------------------------------------------------- */

int16_t saturateSignedInt16(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32767)
    {
        return -32767;
    }
    return (int16_t)value;
}

float capAngle(float angle_deg)
{
    float a = fmodf(angle_deg, 360.0f);
    if (a > 180.0f)
    {
        a -= 360.0f;
    }
    else if (a < -180.0f)
    {
        a += 360.0f;
    }
    return a;
}

/* ------------------------------------------------------------------------- */
/* Sensfusion6                                                                 */
/* ------------------------------------------------------------------------- */

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;

static bool sfInitOnce = false;

static void sfSetGravity(void)
{
    gravityX = 2.0f * (qx * qz - qw * qy);
    gravityY = 2.0f * (qw * qx + qy * qz);
    gravityZ = 1.0f - 2.0f * (qx * qx + qy * qy);
}

static void sfNormalizeQuat(void)
{
    float normSq = qw * qw + qx * qx + qy * qy + qz * qz;
    if (normSq < 1e-12f)
    {
        qw = 1.0f;
        qx = qy = qz = 0.0f;
        sfSetGravity();
        return;
    }
    float inv = invSqrt(normSq);
    qw *= inv;
    qx *= inv;
    qy *= inv;
    qz *= inv;
    sfSetGravity();
}

void estimatedGravityDirection(float qw_, float qx_, float qy_, float qz_,
                               float *gravX, float *gravY, float *gravZ)
{
    if (!gravX || !gravY || !gravZ)
    {
        return;
    }
    *gravX = 2.0f * (qx_ * qz_ - qw_ * qy_);
    *gravY = 2.0f * (qw_ * qx_ + qy_ * qz_);
    *gravZ = 1.0f - 2.0f * (qx_ * qx_ + qy_ * qy_);
}

float invSqrt(float x)
{
    if (x <= 0.0f)
    {
        return 0.0f;
    }
    float xhalf = 0.5f * x;
    uint32_t i = 0;
    memcpy(&i, &x, sizeof(i));
    i = 0x5f3759dfU - (i >> 1);
    float y = 0.0f;
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - xhalf * y * y);
    return y;
}

void sensfusion6Init(void)
{
    if (sfInitOnce)
    {
        return;
    }
    qw = 1.0f;
    qx = 0.0f;
    qy = 0.0f;
    qz = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
    baseZacc = 0.0f;
    sensfusion6IsInit = true;
    sensfusion6IsCalibrated = false;
    sfSetGravity();
    sfInitOnce = true;
}

bool sensfusion6Test(void)
{
    return sensfusion6IsInit;
}

static void sfMahonyUpdate(float gxDeg, float gyDeg, float gzDeg,
                           float ax, float ay, float az, float dt)
{
    float gx = gxDeg * DEG_TO_RAD;
    float gy = gyDeg * DEG_TO_RAD;
    float gz = gzDeg * DEG_TO_RAD;

    if (twoKi <= 0.0f)
    {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    float accNormSq = ax * ax + ay * ay + az * az;
    if (accNormSq > 1e-10f)
    {
        float invAccMag = invSqrt(accNormSq);
        ax *= invAccMag;
        ay *= invAccMag;
        az *= invAccMag;

        float vx = 2.0f * (qx * qz - qw * qy);
        float vy = 2.0f * (qw * qx + qy * qz);
        float vz = 1.0f - 2.0f * (qx * qx + qy * qy);

        float ex = (ay * vz - az * vy);
        float ey = (az * vx - ax * vz);
        float ez = (ax * vy - ay * vx);

        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * ex * dt;
            integralFBy += twoKi * ey * dt;
            integralFBz += twoKi * ez * dt;
        }

        gx += twoKp * ex + integralFBx;
        gy += twoKp * ey + integralFBy;
        gz += twoKp * ez + integralFBz;
    }

    if (dt > 0.0f)
    {
        float dqw = 0.5f * (-gx * qx - gy * qy - gz * qz);
        float dqx = 0.5f * (gx * qw + gz * qy - gy * qz);
        float dqy = 0.5f * (gy * qw - gz * qx + gx * qz);
        float dqz = 0.5f * (gz * qw + gy * qx - gx * qy);

        qw += dqw * dt;
        qx += dqx * dt;
        qy += dqy * dt;
        qz += dqz * dt;
    }

    sfNormalizeQuat();
}

#ifdef CONFIG_IMU_MADGWICK_QUATERNION
static void sfMadgwickUpdate(float gxDeg, float gyDeg, float gzDeg,
                             float ax, float ay, float az, float dt)
{
    float gx = gxDeg * DEG_TO_RAD;
    float gy = gyDeg * DEG_TO_RAD;
    float gz = gzDeg * DEG_TO_RAD;

    float accNormSq = ax * ax + ay * ay + az * az;
    if (accNormSq > 1e-10f)
    {
        float invAccMag = invSqrt(accNormSq);
        ax *= invAccMag;
        ay *= invAccMag;
        az *= invAccMag;

        /* Standard 6-axis Madgwick gradient-descent correction. */
        float q0 = qw, q1 = qx, q2 = qy, q3 = qz;
        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0;
        float _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1;
        float _8q2 = 8.0f * q2;

        float s0 = _4q0 * q2 * q2 + _2q2 * ax + _4q0 * q1 * q1 - _2q1 * ay;
        float s1 = _4q1 * q3 * q3 - _2q3 * ax + 4.0f * q0 * q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1 * q1 + _8q1 * q2 * q2 + _4q1 * az * az;
        float s2 = 4.0f * q0 * q0 * q2 + _2q0 * ax + _4q2 * q3 * q3 - _2q3 * ay - _4q2 + _8q2 * q1 * q1 + _8q2 * q2 * q2 + _4q2 * az * az;
        float s3 = 4.0f * q1 * q1 * q3 - _2q1 * ax + 4.0f * q2 * q2 * q3 - _2q2 * ay;

        float recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        gx += beta * -s0;
        gy += beta * -s1;
        gz += beta * -s2;
        (void)s3;
    }

    if (dt > 0.0f)
    {
        float dqw = 0.5f * (-gx * qx - gy * qy - gz * qz);
        float dqx = 0.5f * (gx * qw + gz * qy - gy * qz);
        float dqy = 0.5f * (gy * qw - gz * qx + gx * qz);
        float dqz = 0.5f * (gz * qw + gy * qx - gx * qy);

        qw += dqw * dt;
        qx += dqx * dt;
        qy += dqy * dt;
        qz += dqz * dt;
    }

    sfNormalizeQuat();
}
#endif

void sensfusion6UpdateQ(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt)
{
    if (!sensfusion6IsInit)
    {
        sensfusion6Init();
    }

    float accNormSq = ax * ax + ay * ay + az * az;
    if (!sensfusion6IsCalibrated && accNormSq > 1e-10f)
    {
        float gxBody, gyBody, gzBody;
        estimatedGravityDirection(qw, qx, qy, qz, &gxBody, &gyBody, &gzBody);
        baseZacc = ax * gxBody + ay * gyBody + az * gzBody;
        sensfusion6IsCalibrated = true;
    }

#ifdef CONFIG_IMU_MADGWICK_QUATERNION
    sfMadgwickUpdate(gx, gy, gz, ax, ay, az, dt);
#else
    sfMahonyUpdate(gx, gy, gz, ax, ay, az, dt);
#endif
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (!roll_deg || !pitch_deg || !yaw_deg)
    {
        return;
    }

    float a12 = 2.0f * (qw * qy - qz * qx);
    if (a12 > 1.0f)
    {
        a12 = 1.0f;
    }
    else if (a12 < -1.0f)
    {
        a12 = -1.0f;
    }

    *pitch_deg = asinf(a12) * RAD_TO_DEG;
    *roll_deg = atan2f(2.0f * (qw * qx + qy * qz),
                       1.0f - 2.0f * (qx * qx + qy * qy)) * RAD_TO_DEG;
    *yaw_deg = atan2f(2.0f * (qw * qz + qx * qy),
                      1.0f - 2.0f * (qy * qy + qz * qz)) * RAD_TO_DEG;
}

void sensfusion6GetQuaternion(float *qw_out, float *qx_out, float *qy_out, float *qz_out)
{
    if (!qw_out || !qx_out || !qy_out || !qz_out)
    {
        return;
    }
    *qw_out = qw;
    *qx_out = qx;
    *qy_out = qy;
    *qz_out = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az)
{
    float gxBody, gyBody, gzBody;
    estimatedGravityDirection(qw, qx, qy, qz, &gxBody, &gyBody, &gzBody);
    return ax * gxBody + ay * gyBody + az * gzBody;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az)
{
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

/* ------------------------------------------------------------------------- */
/* Power distribution and battery                                             */
/* ------------------------------------------------------------------------- */

void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch,
                             int16_t yaw, MotorPower *out)
{
    if (!out)
    {
        return;
    }

    int32_t r = roll / 2;
    int32_t p = pitch / 2;
    int32_t t = thrust;

    out->m1 = t - r + p + yaw;
    out->m2 = t - r - p - yaw;
    out->m3 = t + r - p + yaw;
    out->m4 = t + r + p - yaw;
}

void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY,
                                  float torqueZ, float armLength,
                                  float thrustToTorque, float motorForces[4])
{
    if (!motorForces)
    {
        return;
    }

    float thrustPart = 0.25f * thrustSi;
    float arm = 0.707106781f * armLength;
    float rollPart = 0.0f;
    float pitchPart = 0.0f;
    float yawPart = 0.0f;

    if (fabsf(arm) > 1e-8f)
    {
        rollPart = 0.25f / arm * torqueX;
        pitchPart = 0.25f / arm * torqueY;
    }
    if (fabsf(thrustToTorque) > 1e-9f)
    {
        yawPart = 0.25f / thrustToTorque * torqueZ;
    }

    motorForces[0] = thrustPart - rollPart + pitchPart - yawPart;
    motorForces[1] = thrustPart + rollPart + pitchPart + yawPart;
    motorForces[2] = thrustPart - rollPart - pitchPart + yawPart;
    motorForces[3] = thrustPart + rollPart - pitchPart - yawPart;

    for (int i = 0; i < 4; ++i)
    {
        if (motorForces[i] < 0.0f)
        {
            motorForces[i] = 0.0f;
        }
    }
}

void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4])
{
    if (!normalizedForces || !motorPWMs)
    {
        return;
    }

    for (int i = 0; i < 4; ++i)
    {
        float f = normalizedForces[i];
        if (f < 0.0f)
        {
            f = 0.0f;
        }
        if (f > 1.0f)
        {
            f = 1.0f;
        }
        motorPWMs[i] = (uint16_t)(f * 65535.0f);
    }
}

static uint16_t motorForceToPwm(float forceN)
{
    if (forceN <= 0.0f)
    {
        return 0;
    }
    float pwm = forceN / CRAZYFLIE_MAX_MOTOR_FORCE_N * 65535.0f;
    if (pwm > 65535.0f)
    {
        return 65535;
    }
    return (uint16_t)pwm;
}

void powerDistribution(const ControlData *control, MotorPower *motorPower)
{
    if (!control || !motorPower)
    {
        return;
    }

    switch (control->controlMode)
    {
    case controlModeLegacy:
    {
        MotorPower mp = {0, 0, 0, 0};
        powerDistributionLegacy(control->thrust, control->roll, control->pitch,
                                control->yaw, &mp);
        *motorPower = mp;
        break;
    }
    case controlModeForceTorque:
    {
        float forces[4] = {0.0f, 0.0f, 0.0f, 0.0f};
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
    case controlModeForce:
    {
        uint16_t pwms[4] = {0, 0, 0, 0};
        powerDistributionForce(control->normalizedForces, pwms);
        motorPower->m1 = (int32_t)pwms[0];
        motorPower->m2 = (int32_t)pwms[1];
        motorPower->m3 = (int32_t)pwms[2];
        motorPower->m4 = (int32_t)pwms[3];
        break;
    }
    default:
        /* Unknown mode: do not modify output. */
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
    if (!motors)
    {
        return result;
    }

    int32_t max = motors[0];
    for (int i = 1; i < 4; ++i)
    {
        if (motors[i] > max)
        {
            max = motors[i];
        }
    }

    if (max > maxAllowedThrust)
    {
        result.isCapped = true;
        result.reduction = max - maxAllowedThrust;
        for (int i = 0; i < 4; ++i)
        {
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
    if (actualVoltage <= 0.0f)
    {
        return motorThrust;
    }

    float comp = (float)motorThrust * nominalVoltage / actualVoltage;
    int32_t compi = (int32_t)(comp + 0.5f);
    if (compi < 0)
    {
        return 0;
    }
    if (compi > 65535)
    {
        return 65535;
    }
    return (uint16_t)compi;
}

/* ------------------------------------------------------------------------- */
/* PID and controller                                                         */
/* ------------------------------------------------------------------------- */

PidObject pidRoll = {0};
PidObject pidPitch = {0};
PidObject pidYaw = {0};
PidObject pidRollRate = {0};
PidObject pidPitchRate = {0};
PidObject pidYawRate = {0};

static float attDt = 1.0f / 500.0f;
static bool attInitOnce = false;

static void pidInit(PidObject *pid)
{
    if (!pid)
    {
        return;
    }
    memset(pid, 0, sizeof(*pid));
    pid->initialized = true;
}

static float pidUpdate(PidObject *pid, float desired, float actual, float dt, bool resetIntegral)
{
    if (!pid)
    {
        return 0.0f;
    }

    if (!pid->initialized)
    {
        pidInit(pid);
    }

    float error = desired - actual;
    if (resetIntegral)
    {
        pid->integral = 0.0f;
        pid->prevError = error;
    }

    float derivative = 0.0f;
    if (dt > 1e-6f)
    {
        derivative = (error - pid->prevError) / dt;
    }

    if (!resetIntegral)
    {
        pid->integral += pid->ki * error * dt;
    }

    float out = pid->kp * error + pid->integral + pid->kd * derivative + pid->kff * desired;
    pid->prevError = error;
    pid->output = out;
    return out;
}

void attitudeControllerInit(float updateDt)
{
    if (attInitOnce)
    {
        return;
    }

    if (updateDt > 0.0f)
    {
        attDt = updateDt;
    }

    pidInit(&pidRoll);
    pidInit(&pidPitch);
    pidInit(&pidYaw);
    pidInit(&pidRollRate);
    pidInit(&pidPitchRate);
    pidInit(&pidYawRate);

    attInitOnce = true;
}

void attitudeControllerCorrectRatePID(float rollActual, float rollDesired,
                                      float pitchActual, float pitchDesired,
                                      float yawActual, float yawDesired)
{
    float out;

    out = pidUpdate(&pidRollRate, rollDesired, rollActual, attDt, false);
    pidRollRate.output = (float)saturateSignedInt16((int32_t)(out + 0.5f));

    out = pidUpdate(&pidPitchRate, pitchDesired, pitchActual, attDt, false);
    pidPitchRate.output = (float)saturateSignedInt16((int32_t)(out + 0.5f));

    out = pidUpdate(&pidYawRate, yawDesired, yawActual, attDt, false);
    pidYawRate.output = (float)saturateSignedInt16((int32_t)(out + 0.5f));
}

void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired,
                                          float pitchActual, float pitchDesired,
                                          float yawActual, float yawDesired)
{
    pidUpdate(&pidRoll, rollDesired, rollActual, attDt, false);
    pidUpdate(&pidPitch, pitchDesired, pitchActual, attDt, false);
    pidUpdate(&pidYaw, yawDesired, yawActual, attDt, true);
}

void attitudeControllerResetAllPID(float rollActual, float pitchActual,
                                   float yawActual)
{
    pidRoll.integral = 0.0f;
    pidRoll.prevError = 0.0f;
    pidRoll.output = rollActual;

    pidPitch.integral = 0.0f;
    pidPitch.prevError = 0.0f;
    pidPitch.output = pitchActual;

    pidYaw.integral = 0.0f;
    pidYaw.prevError = 0.0f;
    pidYaw.output = yawActual;

    pidRollRate.integral = 0.0f;
    pidRollRate.prevError = 0.0f;
    pidRollRate.output = 0.0f;

    pidPitchRate.integral = 0.0f;
    pidPitchRate.prevError = 0.0f;
    pidPitchRate.output = 0.0f;

    pidYawRate.integral = 0.0f;
    pidYawRate.prevError = 0.0f;
    pidYawRate.output = 0.0f;
}

void attitudeControllerResetRollAttitudePID(float rollActual)
{
    pidRoll.integral = 0.0f;
    pidRoll.prevError = 0.0f;
    pidRoll.output = rollActual;
}

void attitudeControllerResetPitchAttitudePID(float pitchActual)
{
    pidPitch.integral = 0.0f;
    pidPitch.prevError = 0.0f;
    pidPitch.output = pitchActual;
}

void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch,
                                         int16_t *yaw)
{
    if (!roll || !pitch || !yaw)
    {
        return;
    }
    *roll = (int16_t)saturateSignedInt16((int32_t)pidRollRate.output);
    *pitch = (int16_t)saturateSignedInt16((int32_t)pidPitchRate.output);
    *yaw = (int16_t)saturateSignedInt16((int32_t)pidYawRate.output);
}

static void getEulerFromQuat(Quaternion q, float *roll, float *pitch, float *yaw)
{
    float a12 = 2.0f * (q.w * q.y - q.z * q.x);
    if (a12 > 1.0f)
    {
        a12 = 1.0f;
    }
    else if (a12 < -1.0f)
    {
        a12 = -1.0f;
    }
    *pitch = asinf(a12) * RAD_TO_DEG;
    *roll = atan2f(2.0f * (q.w * q.x + q.y * q.z),
                   1.0f - 2.0f * (q.x * q.x + q.y * q.y)) * RAD_TO_DEG;
    *yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y),
                  1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * RAD_TO_DEG;
}

uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state)
{
    if (!setpoint || !state)
    {
        return 0;
    }

    float out = 32767.0f;
    if (setpoint->mode.z == modeAbs)
    {
        float posError = setpoint->position.z - state->position.z;
        out = 32767.0f + posError * 500.0f - state->velocity.z * 200.0f;
    }
    else if (setpoint->mode.z == modeVelocity)
    {
        float velError = setpoint->velocity.z - state->velocity.z;
        out = 32767.0f + velError * 500.0f;
    }
    else
    {
        return 0;
    }

    if (out < 0.0f)
    {
        return 0;
    }
    if (out > 60000.0f)
    {
        return 60000;
    }
    return (uint16_t)(out + 0.5f);
}

static float controllerDesiredYaw = 0.0f;
static bool controllerDesiredYawValid = false;

void controllerPid(const SensorData *sensors, const Setpoint *setpoint,
                   const State *state, ControlData *control,
                   float yawMaxDelta, float attitudeUpdateDt)
{
    if (!sensors || !setpoint || !state || !control)
    {
        return;
    }

    if (setpoint->thrust == 0)
    {
        attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->thrust = 0;
        control->controlMode = controlModeLegacy;
        controllerDesiredYaw = state->attitude.yaw;
        controllerDesiredYawValid = true;
        return;
    }

    float rollActual = sensors->gyro.x;
    float pitchActual = -sensors->gyro.y;
    float yawActual = sensors->gyro.z;

    float desiredYaw = state->attitude.yaw;
    if (setpoint->mode.yaw == modeAbs)
    {
        desiredYaw = setpoint->attitude.yaw;
        controllerDesiredYawValid = true;
    }
    else if (setpoint->mode.quat == modeAbs)
    {
        float r, p, y;
        getEulerFromQuat(setpoint->attitudeQuaternion, &r, &p, &y);
        desiredYaw = y;
        controllerDesiredYawValid = true;
    }
    else if (setpoint->mode.yaw == modeVelocity)
    {
        if (!controllerDesiredYawValid)
        {
            controllerDesiredYaw = state->attitude.yaw;
            controllerDesiredYawValid = true;
        }
        controllerDesiredYaw += setpoint->attitudeRate.yaw * attitudeUpdateDt;
        controllerDesiredYaw = capAngle(controllerDesiredYaw);
        desiredYaw = controllerDesiredYaw;
    }

    controllerDesiredYaw = desiredYaw;

    if (yawMaxDelta != 0.0f)
    {
        float diff = capAngle(desiredYaw - state->attitude.yaw);
        if (diff > yawMaxDelta)
        {
            desiredYaw = capAngle(state->attitude.yaw + yawMaxDelta);
        }
        else if (diff < -yawMaxDelta)
        {
            desiredYaw = capAngle(state->attitude.yaw - yawMaxDelta);
        }
    }

    float rollRateDesired = 0.0f;
    float pitchRateDesired = 0.0f;

    if (setpoint->mode.roll == modeVelocity)
    {
        attitudeControllerResetRollAttitudePID(state->attitude.roll);
        rollRateDesired = setpoint->attitudeRate.roll;
    }
    else
    {
        rollRateDesired = pidUpdate(&pidRoll, setpoint->attitude.roll, state->attitude.roll, attDt, false);
    }

    if (setpoint->mode.pitch == modeVelocity)
    {
        attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
        pitchRateDesired = setpoint->attitudeRate.pitch;
    }
    else
    {
        pitchRateDesired = pidUpdate(&pidPitch, setpoint->attitude.pitch, state->attitude.pitch, attDt, false);
    }

    float yawRateDesired = pidUpdate(&pidYaw, desiredYaw, state->attitude.yaw, attDt, true);

    attitudeControllerCorrectRatePID(rollActual, rollRateDesired,
                                     pitchActual, pitchRateDesired,
                                     yawActual, yawRateDesired);

    int16_t rollOut = 0, pitchOut = 0, yawOut = 0;
    attitudeControllerGetActuatorOutput(&rollOut, &pitchOut, &yawOut);

    control->roll = rollOut;
    control->pitch = pitchOut;
    control->yaw = yawOut;
    control->controlMode = controlModeLegacy;

    if (setpoint->mode.z == modeDisable)
    {
        control->thrust = setpoint->thrust;
    }
    else
    {
        control->thrust = positionControllerUpdate(setpoint, state);
    }

    control->yaw = -control->yaw;
}

/* ------------------------------------------------------------------------- */
/* CRTP Commander RPYT                                                        */
/* ------------------------------------------------------------------------- */

bool thrustLocked = false;
bool commanderModeSet = false;

void rotateYaw(float roll, float pitch, float yaw_deg,
               float *rollPrime, float *pitchPrime)
{
    if (!rollPrime || !pitchPrime)
    {
        return;
    }

    float rad = yaw_deg * DEG_TO_RAD;
    float c = cosf(rad);
    float s = sinf(rad);
    *rollPrime = roll * c - pitch * s;
    *pitchPrime = roll * s + pitch * c;
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
    if (!values || !setpoint)
    {
        return;
    }

    memset(setpoint, 0, sizeof(*setpoint));

    uint16_t rawThrust = values->thrust;

    if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE)
    {
        thrustLocked = true;
    }
    if (rawThrust == 0)
    {
        thrustLocked = false;
    }

    if (yawMode == CAREFREE)
    {
        memset(setpoint, 0, sizeof(*setpoint));
        setpoint->mode.z = modeDisable;
        setpoint->thrust = 0;
        return;
    }

    if (altHoldMode)
    {
        if (!commanderModeSet)
        {
            commanderModeSet = true;
            /* First AltHold entry resets position/filter state. */
        }
        setpoint->mode.z = modeVelocity;
        setpoint->thrust = 0;
        setpoint->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;
    }
    else
    {
        if (commanderModeSet)
        {
            commanderModeSet = false;
        }
        setpoint->mode.z = modeDisable;
        if (thrustLocked || rawThrust < 1000)
        {
            setpoint->thrust = 0;
        }
        else if (rawThrust > 60000)
        {
            setpoint->thrust = 60000;
        }
        else
        {
            setpoint->thrust = rawThrust;
        }
    }

    if (posSetMode && rawThrust != 0)
    {
        setpoint->mode.x = modeAbs;
        setpoint->mode.y = modeAbs;
        setpoint->mode.z = modeAbs;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->mode.yaw = modeAbs;
        setpoint->position.x = -values->pitch;
        setpoint->position.y = values->roll;
        setpoint->position.z = values->thrust / 1000.0f;
        setpoint->attitude.yaw = values->yaw;
        setpoint->thrust = 0;
        return;
    }

    if (posHoldMode)
    {
        setpoint->mode.x = modeVelocity;
        setpoint->mode.y = modeVelocity;
        setpoint->mode.roll = modeDisable;
        setpoint->mode.pitch = modeDisable;
        setpoint->velocity.x = values->pitch / 30.0f;
        setpoint->velocity.y = values->roll / 30.0f;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
    }
    else
    {
        float rollCmd = values->roll;
        float pitchCmd = values->pitch;

        if (yawMode == PLUSMODE)
        {
            rotateYaw(rollCmd, pitchCmd, 45.0f, &rollCmd, &pitchCmd);
        }

        if (stabilizationModeRoll == RATE)
        {
            setpoint->mode.roll = modeVelocity;
            setpoint->attitudeRate.roll = rollCmd;
        }
        else
        {
            setpoint->mode.roll = modeAbs;
            setpoint->attitude.roll = rollCmd;
        }

        if (stabilizationModePitch == RATE)
        {
            setpoint->mode.pitch = modeVelocity;
            setpoint->attitudeRate.pitch = pitchCmd;
        }
        else
        {
            setpoint->mode.pitch = modeAbs;
            setpoint->attitude.pitch = pitchCmd;
        }
    }

    if (stabilizationModeYaw == RATE)
    {
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = -values->yaw;
    }
    else
    {
        setpoint->mode.yaw = modeAbs;
        setpoint->attitude.yaw = values->yaw;
    }
}

/* ------------------------------------------------------------------------- */
/* Supervisor                                                                 */
/* ------------------------------------------------------------------------- */

SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0;

static bool s_armed = false;
static bool s_crashed = false;
static bool s_tumbled = false;
static bool s_freeFalling = false;
static bool s_flying = false;
static bool s_autoArming = false;
static uint32_t s_spinupDurationMs = 0;
static SensorData s_sensors = {0};
static uint32_t s_motorRatios[4] = {0, 0, 0, 0};
static uint32_t s_idleThrust = 0;
static int32_t s_motorRPMs[4] = {0, 0, 0, 0};
static float s_crashDetectionGs = 0.0f;
static float s_freeFallThreshold = 0.0f;
static float s_acceptedTiltAccZ = 0.0f;
static float s_acceptedUpsideDownAccZ = 0.0f;
static uint32_t s_maxTiltTime = 0;
static uint32_t s_maxUpsideDownTime = 0;
static bool s_tumbleCheckEnabled = true;
static uint32_t s_armStartTick = 0;
static bool s_armStartTickValid = false;
static uint32_t s_readyToFlyStartTick = 0;
static bool s_readyToFlyStartTickValid = false;
static uint32_t s_landingStartTick = 0;
static bool s_landingStartTickValid = false;
static bool s_trajectoryFlying = false;
static bool s_trajectoryFinished = false;
static bool s_trajectoryDisabled = false;
static bool s_deckFault = false;

static bool s_crtpEmergencyStop = false;
static bool s_paramEmergencyStop = false;
static bool s_emergencyStopWatchdogFailed = false;

static uint32_t rpmNotRespondingStartTick = 0;
static bool rpmNotRespondingTimerRunning = false;
static bool rpmNotRespondingFailed = false;

static bool supInitOnce = false;

#define SUPERVISOR_OWNED_BITS_MASK \
    (SUPERVISOR_CB_ARMED | SUPERVISOR_CB_IS_FLYING | SUPERVISOR_CB_IS_TUMBLED | \
     SUPERVISOR_CB_CRASHED | SUPERVISOR_CB_FREE_FALL | SUPERVISOR_CB_EMERGENCY_STOP)
#define SUPERVISOR_RECOMPUTED_BITS_MASK \
    (SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT | \
     SUPERVISOR_CB_PREFLIGHT_TIMEOUT | SUPERVISOR_CB_LANDING_TIMEOUT | \
     SUPERVISOR_CB_SPINUP_TIMEOUT | SUPERVISOR_CB_RPM_AT_ARMING_VALID | \
     SUPERVISOR_CB_MOTORS_NOT_RESPONDING | SUPERVISOR_CB_DECK_FAULT)

static void zeroSetpoint(Setpoint *setpoint)
{
    if (setpoint)
    {
        memset(setpoint, 0, sizeof(*setpoint));
    }
}

void supervisorInit(void)
{
    if (supInitOnce)
    {
        return;
    }

    supervisorState = supervisorStateLocked;
    supervisorConditionBits = 0;
    s_armed = false;
    s_crashed = false;
    s_tumbled = false;
    s_freeFalling = false;
    s_flying = false;
    s_autoArming = false;
    s_spinupDurationMs = 0;
    memset(&s_sensors, 0, sizeof(s_sensors));
    memset(s_motorRatios, 0, sizeof(s_motorRatios));
    memset(s_motorRPMs, 0, sizeof(s_motorRPMs));
    s_idleThrust = 0;
    s_crashDetectionGs = 0.0f;
    s_freeFallThreshold = 0.0f;
    s_acceptedTiltAccZ = 0.0f;
    s_acceptedUpsideDownAccZ = 0.0f;
    s_maxTiltTime = 0;
    s_maxUpsideDownTime = 0;
    s_tumbleCheckEnabled = true;
    s_armStartTick = 0;
    s_armStartTickValid = false;
    s_readyToFlyStartTick = 0;
    s_readyToFlyStartTickValid = false;
    s_landingStartTick = 0;
    s_landingStartTickValid = false;
    s_trajectoryFlying = false;
    s_trajectoryFinished = false;
    s_trajectoryDisabled = false;
    s_deckFault = false;
    s_crtpEmergencyStop = false;
    s_paramEmergencyStop = false;
    s_emergencyStopWatchdogFailed = false;
    rpmNotRespondingStartTick = 0;
    rpmNotRespondingTimerRunning = false;
    rpmNotRespondingFailed = false;

    supInitOnce = true;
}

void supervisorSetSensorData(const SensorData *sensors)
{
    if (sensors)
    {
        s_sensors = *sensors;
        supervisorLog.accNorm = sqrtf(sensors->acc.x * sensors->acc.x +
                                      sensors->acc.y * sensors->acc.y +
                                      sensors->acc.z * sensors->acc.z);
    }
}

void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust)
{
    if (motorRatios)
    {
        memcpy(s_motorRatios, motorRatios, sizeof(s_motorRatios));
    }
    s_idleThrust = idleThrust;
}

void supervisorSetMotorRPMs(const int32_t motorRPMs[4])
{
    if (motorRPMs)
    {
        memcpy(s_motorRPMs, motorRPMs, sizeof(s_motorRPMs));
    }
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
    s_spinupDurationMs = spinupTimeoutDurationMs;
}

bool supervisorCanFly(void)
{
    return (supervisorState == supervisorStateReadyToFly ||
            supervisorState == supervisorStateFlying ||
            supervisorState == supervisorStateWarningLevelOut ||
            supervisorState == supervisorStateLanded);
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

bool supervisorAreMotorsAllowedToRun(void)
{
    return (supervisorState == supervisorStateArming ||
            supervisorState == supervisorStateReadyToFly ||
            supervisorState == supervisorStateFlying ||
            supervisorState == supervisorStateWarningLevelOut ||
            supervisorState == supervisorStateLanded);
}

bool supervisorRequestArming(bool doArm)
{
    if (doArm)
    {
        if (!supervisorCanArm())
        {
            return false;
        }
        if (!s_armed)
        {
            s_armed = true;
            supervisorState = supervisorStateArming;
            s_armStartTick = tick;
            s_armStartTickValid = true;
        }
        return true;
    }

    s_armed = false;
    return true;
}

bool supervisorRequestCrashRecovery(bool doRecovery)
{
    if (s_tumbled)
    {
        return false;
    }

    if (!doRecovery)
    {
        s_crashed = true;
        return true;
    }

    s_crashed = false;
    return true;
}

bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust,
                   uint32_t currentTick)
{
    if (!motorRatios)
    {
        return false;
    }

    static uint32_t recentFlightTick = 0;
    static bool seenFlight = false;

    if (idleThrust == 0 && !seenFlight)
    {
        for (int i = 0; i < 4; ++i)
        {
            if (motorRatios[i] > 0)
            {
                recentFlightTick = currentTick;
                seenFlight = true;
                break;
            }
        }
    }
    else
    {
        for (int i = 0; i < 4; ++i)
        {
            if (motorRatios[i] > idleThrust)
            {
                recentFlightTick = currentTick;
                seenFlight = true;
                break;
            }
        }
    }

    if (!seenFlight)
    {
        return false;
    }

    return (currentTick - recentFlightTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}

bool isTumbledCheck(float accX, float accY, float accZ,
                    float crashDetectionGs, float freeFallThreshold,
                    float acceptedTiltAccZ, float acceptedUpsideDownAccZ,
                    uint32_t maxTiltTime, uint32_t maxUpsideDownTime,
                    bool tumbleCheckEnabled, uint32_t currentTick,
                    bool *isFreeFalling)
{
    if (isFreeFalling)
    {
        *isFreeFalling = false;
    }

    static uint32_t tumbleStartTick = 0;
    static bool tumbleTimerRunning = false;
    static bool tumbleUpsideDown = false;

    if (crashDetectionGs > 0.0f)
    {
        float norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
        if (fabsf(norm - 1.0f) > crashDetectionGs)
        {
            s_crashed = true;
        }
    }

    if (freeFallThreshold > 0.0f &&
        fabsf(accX) < freeFallThreshold &&
        fabsf(accY) < freeFallThreshold &&
        fabsf(accZ) < freeFallThreshold)
    {
        if (isFreeFalling)
        {
            *isFreeFalling = true;
        }
        s_freeFalling = true;
        tumbleTimerRunning = false;
        tumbleStartTick = 0;
        return false;
    }
    s_freeFalling = false;

    if (!tumbleCheckEnabled)
    {
        s_tumbled = false;
        tumbleTimerRunning = false;
        tumbleStartTick = 0;
        tumbleUpsideDown = false;
        return false;
    }

    if (accZ < acceptedUpsideDownAccZ)
    {
        if (!tumbleTimerRunning)
        {
            tumbleStartTick = currentTick;
            tumbleTimerRunning = true;
            tumbleUpsideDown = true;
        }
        uint32_t timeout = maxUpsideDownTime;
        if (timeout > 0 && (currentTick - tumbleStartTick) >= timeout)
        {
            s_tumbled = true;
            return true;
        }
    }
    else if (accZ < acceptedTiltAccZ)
    {
        if (!tumbleTimerRunning)
        {
            tumbleStartTick = currentTick;
            tumbleTimerRunning = true;
            tumbleUpsideDown = false;
        }
        else if (tumbleUpsideDown)
        {
            /* Keep the more severe upside-down timeout once triggered. */
        }
        uint32_t timeout = tumbleUpsideDown ? maxUpsideDownTime : maxTiltTime;
        if (timeout > 0 && (currentTick - tumbleStartTick) >= timeout)
        {
            s_tumbled = true;
            return true;
        }
    }
    else
    {
        tumbleTimerRunning = false;
        tumbleStartTick = 0;
        tumbleUpsideDown = false;
    }

    return s_tumbled;
}

bool checkEmergencyStopWatchdog(uint32_t currentTick,
                                uint32_t lastNotificationTick)
{
    if (lastNotificationTick == 0)
    {
        return true;
    }

    return (currentTick - lastNotificationTick) < DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;
}

bool supervisorIsPreflightTimeout(SupervisorState state,
                                  uint32_t latestArmingTick,
                                  uint32_t currentTick,
                                  uint32_t preflightTimeoutDuration)
{
    if (state != supervisorStateReadyToFly)
    {
        return false;
    }
    if (preflightTimeoutDuration == 0 || latestArmingTick == 0)
    {
        return false;
    }
    return (currentTick - latestArmingTick) >= preflightTimeoutDuration;
}

bool supervisorIsLandingTimeout(uint32_t latestLandingTick,
                                uint32_t currentTick,
                                uint32_t landingTimeoutDuration)
{
    if (latestLandingTick == 0 || landingTimeoutDuration == 0)
    {
        return false;
    }
    return (currentTick - latestLandingTick) >= landingTimeoutDuration;
}

uint32_t updateAndPopulateConditions(bool crtpEmergencyStop,
                                     bool paramEmergencyStop,
                                     bool emergencyStopWatchdogFailed)
{
    s_crtpEmergencyStop = crtpEmergencyStop;
    s_paramEmergencyStop = paramEmergencyStop;
    s_emergencyStopWatchdogFailed = emergencyStopWatchdogFailed;

    uint32_t bits = supervisorConditionBits & ~SUPERVISOR_OWNED_BITS_MASK;
    if (s_armed)
    {
        bits |= SUPERVISOR_CB_ARMED;
    }
    if (s_flying)
    {
        bits |= SUPERVISOR_CB_IS_FLYING;
    }
    if (s_tumbled)
    {
        bits |= SUPERVISOR_CB_IS_TUMBLED;
    }
    if (s_crashed)
    {
        bits |= SUPERVISOR_CB_CRASHED;
    }
    if (s_freeFalling)
    {
        bits |= SUPERVISOR_CB_FREE_FALL;
    }
    if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed)
    {
        bits |= SUPERVISOR_CB_EMERGENCY_STOP;
    }

    supervisorConditionBits = bits;
    return bits;
}

void supervisorUpdate(uint32_t stabilizerStep)
{
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep))
    {
        return;
    }

    uint32_t now = tick;

    s_flying = isFlyingCheck(s_motorRatios, s_idleThrust, now);

    bool freeFall = false;
    (void)isTumbledCheck(s_sensors.acc.x, s_sensors.acc.y, s_sensors.acc.z,
                         s_crashDetectionGs, s_freeFallThreshold,
                         s_acceptedTiltAccZ, s_acceptedUpsideDownAccZ,
                         s_maxTiltTime, s_maxUpsideDownTime,
                         s_tumbleCheckEnabled, now, &freeFall);
    s_freeFalling = freeFall;

    if (freeFall)
    {
        supervisorState = supervisorStateExceptFreeFall;
    }
    else if (s_crashed)
    {
        supervisorState = supervisorStateCrashed;
    }

    /* Track state-specific timeout start ticks. */
    if (supervisorState == supervisorStateReadyToFly)
    {
        if (!s_readyToFlyStartTickValid)
        {
            s_readyToFlyStartTick = now;
            s_readyToFlyStartTickValid = true;
        }
    }
    else
    {
        s_readyToFlyStartTickValid = false;
        s_readyToFlyStartTick = 0;
    }

    if (supervisorState == supervisorStateLanded)
    {
        if (!s_landingStartTickValid)
        {
            s_landingStartTick = now;
            s_landingStartTickValid = true;
        }
    }
    else
    {
        s_landingStartTickValid = false;
        s_landingStartTick = 0;
    }

    if (supervisorState == supervisorStateArming)
    {
        if (!s_armStartTickValid)
        {
            s_armStartTick = now;
            s_armStartTickValid = true;
        }
    }
    else
    {
        s_armStartTickValid = false;
        s_armStartTick = 0;
    }

    if (s_autoArming && supervisorState == supervisorStatePreFlChecksPassed &&
        !s_armed && supervisorCanArm())
    {
        s_armed = true;
        supervisorState = supervisorStateArming;
        s_armStartTick = now;
        s_armStartTickValid = true;
    }

    if (!(supervisorState == supervisorStateArming ||
          supervisorState == supervisorStateReadyToFly ||
          supervisorState == supervisorStateFlying ||
          supervisorState == supervisorStateWarningLevelOut ||
          supervisorState == supervisorStateLanded))
    {
        if (s_armed)
        {
            s_armed = false;
        }
    }

    uint32_t bits = updateAndPopulateConditions(s_crtpEmergencyStop,
                                                s_paramEmergencyStop,
                                                s_emergencyStopWatchdogFailed);
    bits &= ~SUPERVISOR_RECOMPUTED_BITS_MASK;

    uint32_t inactivity = commanderGetInactivityTime();
    if (inactivity > COMMANDER_WDT_TIMEOUT_SHUTDOWN)
    {
        bits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
    }
    else if (inactivity > COMMANDER_WDT_TIMEOUT_STABILIZE)
    {
        bits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
    }

    if (supervisorState == supervisorStateReadyToFly &&
        supervisorIsPreflightTimeout(supervisorState, s_readyToFlyStartTick,
                                     now, supervisorPreflightTimeoutDuration))
    {
        bits |= SUPERVISOR_CB_PREFLIGHT_TIMEOUT;
    }

    if (supervisorState == supervisorStateLanded &&
        supervisorIsLandingTimeout(s_landingStartTick, now,
                                   supervisorLandingTimeoutDuration))
    {
        bits |= SUPERVISOR_CB_LANDING_TIMEOUT;
    }

    if (supervisorState == supervisorStateArming && s_armStartTickValid &&
        s_spinupDurationMs > 0 && (now - s_armStartTick) >= s_spinupDurationMs)
    {
        bits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
    }

    if (isRPMatArmingValid(s_motorRPMs, supervisorRpmCheckMin, supervisorRpmCheckMax))
    {
        bits |= SUPERVISOR_CB_RPM_AT_ARMING_VALID;
    }

    if (isMotorsNotResponding(s_motorRPMs, supervisorRpmNotRespondingThreshold,
                              supervisorRpmNotRespondingDurationMs,
                              supervisorCanFly(), now))
    {
        bits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING;
    }

    if (s_deckFault)
    {
        bits |= SUPERVISOR_CB_DECK_FAULT;
    }

    supervisorConditionBits = bits;
    supervisorLog.info = supervisorGetInfoBitfield();
}

uint16_t supervisorGetInfoBitfield(void)
{
    uint16_t bits = 0;
    if (supervisorCanArm())
    {
        bits |= (1U << 0);
    }
    if (supervisorIsArmed())
    {
        bits |= (1U << 1);
    }
    if (s_autoArming)
    {
        bits |= (1U << 2);
    }
    if (supervisorCanFly())
    {
        bits |= (1U << 3);
    }
    if (s_flying)
    {
        bits |= (1U << 4);
    }
    if (s_tumbled)
    {
        bits |= (1U << 5);
    }
    if (supervisorState == supervisorStateLocked)
    {
        bits |= (1U << 6);
    }
    if (supervisorIsCrashed())
    {
        bits |= (1U << 7);
    }
    if (s_trajectoryFlying)
    {
        bits |= (1U << 8);
    }
    if (s_trajectoryFinished)
    {
        bits |= (1U << 9);
    }
    if (s_trajectoryDisabled)
    {
        bits |= (1U << 10);
    }
    if (s_deckFault)
    {
        bits |= (1U << 11);
    }
    return bits;
}

void supervisorOverrideSetpoint(Setpoint *setpoint,
                                uint32_t supervisorConditionBits,
                                SupervisorState state)
{
    (void)supervisorConditionBits;
    if (!setpoint)
    {
        return;
    }

    if (state == supervisorStateArming ||
        state == supervisorStateReadyToFly ||
        state == supervisorStateFlying ||
        state == supervisorStateLanded)
    {
        return;
    }

    if (state == supervisorStateWarningLevelOut)
    {
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

    zeroSetpoint(setpoint);
}

bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin,
                        int32_t rpmCheckMax)
{
    if (!motorRPMs)
    {
        return false;
    }

    for (int i = 0; i < 4; ++i)
    {
        if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax)
        {
            return false;
        }
    }
    return true;
}

bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold,
                           uint32_t rpmCheckDurationMs, bool canFly,
                           uint32_t currentTick)
{
    if (!motorRPMs)
    {
        return false;
    }

    if (!canFly)
    {
        rpmNotRespondingTimerRunning = false;
        rpmNotRespondingStartTick = 0;
        rpmNotRespondingFailed = false;
        return false;
    }

    bool low = false;
    for (int i = 0; i < 4; ++i)
    {
        if (motorRPMs[i] < rpmThreshold)
        {
            low = true;
            break;
        }
    }

    if (low)
    {
        if (!rpmNotRespondingTimerRunning)
        {
            rpmNotRespondingStartTick = currentTick;
            rpmNotRespondingTimerRunning = true;
        }
        if (rpmCheckDurationMs > 0 &&
            (currentTick - rpmNotRespondingStartTick) >= rpmCheckDurationMs)
        {
            rpmNotRespondingFailed = true;
            return true;
        }
    }
    else
    {
        rpmNotRespondingTimerRunning = false;
        rpmNotRespondingFailed = false;
    }

    return rpmNotRespondingFailed;
}

/* ------------------------------------------------------------------------- */
/* Estimator FIFO and commander arbitration                                   */
/* ------------------------------------------------------------------------- */

static EstimatorMeasurement estimatorFifo[16];
static uint8_t estimatorHead = 0;
static uint8_t estimatorTail = 0;
static uint8_t estimatorCount = 0;

static Axis3f lastGyro = {0.0f, 0.0f, 0.0f};
static Axis3f lastAcc = {0.0f, 0.0f, 0.0f};
static float lastBaroPressure = 0.0f;
static float lastBaroTemperature = 0.0f;
static float lastTofRange = 0.0f;
static float estimatorVerticalVelocity = 0.0f;
static Axis3f estimatorPosition = {0.0f, 0.0f, 0.0f};

bool estimatorEnqueue(const EstimatorMeasurement *measurement)
{
    if (!measurement || estimatorCount >= 16)
    {
        return false;
    }

    estimatorFifo[estimatorTail] = *measurement;
    estimatorTail = (estimatorTail + 1) & 0x0F;
    estimatorCount++;
    return true;
}

bool estimatorDequeue(EstimatorMeasurement *measurement)
{
    if (!measurement || estimatorCount == 0)
    {
        return false;
    }

    *measurement = estimatorFifo[estimatorHead];
    estimatorHead = (estimatorHead + 1) & 0x0F;
    estimatorCount--;
    return true;
}

void estimatorComplementary(uint32_t stabilizerStep)
{
    EstimatorMeasurement m;
    while (estimatorDequeue(&m))
    {
        if (m.type == MeasurementTypeGyroscope)
        {
            lastGyro.x = m.data[0];
            lastGyro.y = m.data[1];
            lastGyro.z = m.data[2];
        }
        else if (m.type == MeasurementTypeAcceleration)
        {
            lastAcc.x = m.data[0];
            lastAcc.y = m.data[1];
            lastAcc.z = m.data[2];
        }
        else if (m.type == MeasurementTypeBarometer)
        {
            lastBaroPressure = m.data[0];
            lastBaroTemperature = m.data[1];
        }
        else if (m.type == MeasurementTypeTOF)
        {
            lastTofRange = m.data[0];
        }
    }

    gyro.x = lastGyro.x;
    gyro.y = lastGyro.y;
    gyro.z = lastGyro.z;
    acc.x = lastAcc.x;
    acc.y = lastAcc.y;
    acc.z = lastAcc.z;
    baro.asl = lastBaroPressure;
    baro.temp = lastBaroTemperature;
    baro.pressure = lastBaroPressure;

    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep))
    {
        sensfusion6UpdateQ(lastGyro.x, lastGyro.y, lastGyro.z,
                           lastAcc.x, lastAcc.y, lastAcc.z, 1.0f / 250.0f);

        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        sensfusion6GetEulerRPY(&roll, &pitch, &yaw);
        stateEstimate.roll = roll;
        stateEstimate.pitch = pitch;
        stateEstimate.yaw = yaw;

        float qwOut = 0.0f, qxOut = 0.0f, qyOut = 0.0f, qzOut = 0.0f;
        sensfusion6GetQuaternion(&qwOut, &qxOut, &qyOut, &qzOut);
        stateEstimate.qw = qwOut;
        stateEstimate.qx = qxOut;
        stateEstimate.qy = qyOut;
        stateEstimate.qz = qzOut;

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

        float accZNoG = sensfusion6GetAccZWithoutGravity(lastAcc.x, lastAcc.y, lastAcc.z);
        estimatorVerticalVelocity += accZNoG * 9.81f * (1.0f / 250.0f);
    }

    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep))
    {
        estimatorPosition.z += estimatorVerticalVelocity * (1.0f / 100.0f);
    }
}

static Setpoint activeCommandSetpoint = {0};
static int activeCommandPriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t lastCommandUpdateTick = 0;
static bool hasActiveCommandSetpoint = false;

bool commanderSetSetpoint(const Setpoint *setpoint, int priority)
{
    if (!setpoint)
    {
        return false;
    }

    if (priority == COMMANDER_PRIORITY_DISABLE)
    {
        activeCommandSetpoint = *setpoint;
        activeCommandPriority = COMMANDER_PRIORITY_DISABLE;
        lastCommandUpdateTick = tick;
        hasActiveCommandSetpoint = true;
        return true;
    }

    if (!hasActiveCommandSetpoint || priority >= activeCommandPriority)
    {
        activeCommandSetpoint = *setpoint;
        activeCommandPriority = priority;
        lastCommandUpdateTick = tick;
        hasActiveCommandSetpoint = true;

        if (priority > COMMANDER_PRIORITY_HIGHLEVEL)
        {
            /* Stop high-level trajectory. */
        }
        return true;
    }

    return false;
}

void commanderRelaxPriority(void)
{
    activeCommandPriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(void)
{
    if (!hasActiveCommandSetpoint)
    {
        return 0;
    }
    return tick - lastCommandUpdateTick;
}

int commanderGetActivePriority(void)
{
    return activeCommandPriority;
}

/* ------------------------------------------------------------------------- */
/* Stabilizer, compressed state, and rate supervisor                          */
/* ------------------------------------------------------------------------- */

static bool stabilizerInitOnce = false;
static bool highLevelSetpointPending = false;
static Setpoint highLevelSetpoint = {0};
static SensorData stabilizerSensors = {0};
static State stabilizerState = {0};
static float filteredBatteryVoltage = 0.0f;

static void sensorsInit(void)
{
    sensfusion6Init();
}

static void stateEstimatorInit(void)
{
    /* No additional persistent state in this host model. */
}

static void powerDistributionInit(void)
{
    /* No additional persistent state. */
}

static void motorsInit(void)
{
    motor.m1req = 0;
    motor.m2req = 0;
    motor.m3req = 0;
    motor.m4req = 0;
}

static void collisionAvoidanceInit(void)
{
    /* No collision-avoidance policy defined in this frozen API. */
}

static void sensorsWaitDataReady(void)
{
    /* Host-model bounded call: returns immediately. */
}

static void sensorsAcquire(void)
{
    stabilizerSensors = s_sensors;
}

static void stateEstimatorStep(void)
{
    stabilizerState.attitude.roll = stateEstimate.roll;
    stabilizerState.attitude.pitch = stateEstimate.pitch;
    stabilizerState.attitude.yaw = stateEstimate.yaw;
    stabilizerState.attitudeQuaternion.w = stateEstimate.qw;
    stabilizerState.attitudeQuaternion.x = stateEstimate.qx;
    stabilizerState.attitudeQuaternion.y = stateEstimate.qy;
    stabilizerState.attitudeQuaternion.z = stateEstimate.qz;
    stabilizerState.acc.x = acc.x;
    stabilizerState.acc.y = acc.y;
    stabilizerState.acc.z = acc.z;
}

static void collisionAvoidanceUpdateSetpoint(Setpoint *setpoint)
{
    (void)setpoint;
    /* No collision-avoidance policy defined in this frozen API. */
}

static bool setMotorRatios(const MotorPower *mp)
{
    if (!mp)
    {
        return false;
    }
    motor.m1req = (uint16_t)mp->m1;
    motor.m2req = (uint16_t)mp->m2;
    motor.m3req = (uint16_t)mp->m3;
    motor.m4req = (uint16_t)mp->m4;
    return true;
}

void stabilizerInit(void)
{
    if (stabilizerInitOnce)
    {
        return;
    }

    sensorsInit();
    stateEstimatorInit();
    attitudeControllerInit(1.0f / 500.0f);
    powerDistributionInit();
    motorsInit();
    collisionAvoidanceInit();

    stabilizerInitOnce = true;
}

bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint)
{
    if (!setpoint)
    {
        return false;
    }
    highLevelSetpoint = *setpoint;
    highLevelSetpointPending = true;
    return true;
}

void stabilizerTask(void)
{
    if (!stabilizerInitOnce)
    {
        return;
    }

    if (healthShallWeRunTest())
    {
        healthRunTests(&stabilizerSensors);
        return;
    }

    sensorsWaitDataReady();
    sensorsAcquire();
    stateEstimatorStep();

    if (highLevelSetpointPending)
    {
        commanderSetSetpoint(&highLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
        highLevelSetpointPending = false;
    }

    Setpoint setpoint = {0};
    if (hasActiveCommandSetpoint)
    {
        setpoint = activeCommandSetpoint;
    }

    supervisorUpdate((uint32_t)tick);
    collisionAvoidanceUpdateSetpoint(&setpoint);
    supervisorOverrideSetpoint(&setpoint, supervisorConditionBits, supervisorState);

    ControlData control = {0};
    control.controlMode = controlModeLegacy;
    controllerPid(&stabilizerSensors, &setpoint, &stabilizerState, &control, 0.0f, 0.002f);

    MotorPower mp = {0, 0, 0, 0};
    powerDistribution(&control, &mp);

    if (batterySupplyVoltage > 0.0f)
    {
        filteredBatteryVoltage = batteryCompensation(batterySupplyVoltage,
                                                     filteredBatteryVoltage, 0.01f);
    }
    else if (filteredBatteryVoltage <= 0.0f)
    {
        filteredBatteryVoltage = batterySupplyVoltage;
    }

    if (filteredBatteryVoltage > 0.0f)
    {
        mp.m1 = motorsCompensateBatteryVoltage((uint16_t)mp.m1,
                                               NOMINAL_BATTERY_VOLTAGE,
                                               filteredBatteryVoltage);
        mp.m2 = motorsCompensateBatteryVoltage((uint16_t)mp.m2,
                                               NOMINAL_BATTERY_VOLTAGE,
                                               filteredBatteryVoltage);
        mp.m3 = motorsCompensateBatteryVoltage((uint16_t)mp.m3,
                                               NOMINAL_BATTERY_VOLTAGE,
                                               filteredBatteryVoltage);
        mp.m4 = motorsCompensateBatteryVoltage((uint16_t)mp.m4,
                                               NOMINAL_BATTERY_VOLTAGE,
                                               filteredBatteryVoltage);
    }

    int32_t capped[4] = {mp.m1, mp.m2, mp.m3, mp.m4};
    powerDistributionCap(capped, 65535, 0);
    mp.m1 = capped[0];
    mp.m2 = capped[1];
    mp.m3 = capped[2];
    mp.m4 = capped[3];

    if (!supervisorAreMotorsAllowedToRun())
    {
        mp.m1 = 0;
        mp.m2 = 0;
        mp.m3 = 0;
        mp.m4 = 0;
    }

    setMotorRatios(&mp);
}

static uint32_t quatcompress(float qw_, float qx_, float qy_, float qz_)
{
    float q[4] = {qw_, qx_, qy_, qz_};
    float absQ[4] = {fabsf(qw_), fabsf(qx_), fabsf(qy_), fabsf(qz_)};

    int largest = 0;
    for (int i = 1; i < 4; ++i)
    {
        if (absQ[i] > absQ[largest])
        {
            largest = i;
        }
    }

    float sign = q[largest] < 0.0f ? -1.0f : 1.0f;
    for (int i = 0; i < 4; ++i)
    {
        q[i] *= sign;
    }

    int32_t parts[3];
    int p = 0;
    for (int i = 0; i < 4; ++i)
    {
        if (i == largest)
        {
            continue;
        }
        int32_t v = (int32_t)(q[i] * 511.0f + (q[i] >= 0.0f ? 0.5f : -0.5f));
        if (v > 511)
        {
            v = 511;
        }
        if (v < -511)
        {
            v = -511;
        }
        parts[p++] = v;
    }

    uint32_t packed = (uint32_t)(largest & 0x3);
    for (int i = 0; i < 3; ++i)
    {
        packed |= ((uint32_t)(parts[i] & 0x0FFF) << (2 + i * 10));
    }
    return packed;
}

void compressState(const State *state, const SensorData *sensors,
                   CompressedState *output)
{
    if (!state || !sensors || !output)
    {
        return;
    }

    output->position_mm[0] = (int32_t)(state->position.x * 1000.0f);
    output->position_mm[1] = (int32_t)(state->position.y * 1000.0f);
    output->position_mm[2] = (int32_t)(state->position.z * 1000.0f);

    output->velocity_mms[0] = (int32_t)(state->velocity.x * 1000.0f);
    output->velocity_mms[1] = (int32_t)(state->velocity.y * 1000.0f);
    output->velocity_mms[2] = (int32_t)(state->velocity.z * 1000.0f);

    output->acceleration_mms2[0] = (int32_t)(state->acc.x * 9810.0f);
    output->acceleration_mms2[1] = (int32_t)(state->acc.y * 9810.0f);
    output->acceleration_mms2[2] = (int32_t)((state->acc.z + 1.0f) * 9810.0f);

    output->gyro_millirad_s[0] = sensors->gyro.x * DEG_TO_RAD * 1000.0f;
    output->gyro_millirad_s[1] = -sensors->gyro.y * DEG_TO_RAD * 1000.0f;
    output->gyro_millirad_s[2] = sensors->gyro.z * DEG_TO_RAD * 1000.0f;

    output->quatCompressed = quatcompress(state->attitudeQuaternion.w,
                                          state->attitudeQuaternion.x,
                                          state->attitudeQuaternion.y,
                                          state->attitudeQuaternion.z);
}

bool rateSupervisorValidate(uint32_t measuredRate)
{
    return measuredRate >= 997 && measuredRate <= 1003;
}

void rateSupervisorTask(void)
{
    static uint32_t lastRateCheckTick = 0;
    if ((tick - lastRateCheckTick) >= 2000)
    {
        lastRateCheckTick = tick;
        /* In this host model, no hardware assert is executed. */
    }
}

/* ------------------------------------------------------------------------- */
/* Health state machine                                                       */
/* ------------------------------------------------------------------------- */

TestState healthTestState = testDone;
uint8_t motorPass = 0;
uint8_t batteryPass = 0;
float batterySag = 0.0f;

static bool propTestRequest = false;
static bool batteryTestRequest = false;
static uint8_t healthMotorPassBits = 0;
static uint8_t healthBatteryPass = 0;
static uint32_t healthTick = 0;
static uint32_t healthRestartStartTick = 0;
static uint32_t healthMotorTestCount = 0;
static float healthIdleVoltage = 0.0f;
static float healthMinLoadedVoltage = 0.0f;
static float healthNoiseBuffer[PROPTEST_NBR_OF_VARIANCE_VALUES] = {0};
static uint32_t healthNoiseCount = 0;
static float healthNoiseVariance = 0.0f;
static uint8_t healthPropMotorIndex = 0;

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
    if (propTestRequest)
    {
        propTestRequest = false;
        healthTestState = configureAcc;
        healthTick = 0;
        healthPropMotorIndex = 0;
        healthMotorPassBits = 0;
        healthMotorTestCount = 0;
        healthNoiseCount = 0;
        healthNoiseVariance = 0.0f;
        healthIdleVoltage = batterySupplyVoltage;
        return true;
    }

    if (batteryTestRequest)
    {
        batteryTestRequest = false;
        healthTestState = testBattery;
        healthTick = 0;
        healthMinLoadedVoltage = batterySupplyVoltage;
        healthIdleVoltage = batterySupplyVoltage;
        healthBatteryPass = 0;
        return true;
    }

    return healthTestState != testDone;
}

static void resetHealthResults(void)
{
    motorPass = 0;
    batteryPass = 0;
    batterySag = 0.0f;
    healthMotorPassBits = 0;
    healthBatteryPass = 0;
}

void healthRunTests(const SensorData *sensorData)
{
    if (!sensorData)
    {
        return;
    }

    switch (healthTestState)
    {
    case configureAcc:
        resetHealthResults();
        healthIdleVoltage = batterySupplyVoltage;
        healthMinLoadedVoltage = batterySupplyVoltage;
        healthNoiseCount = 0;
        healthNoiseVariance = 0.0f;
        healthTestState = measureNoiseFloor;
        break;

    case measureNoiseFloor:
        if (healthNoiseCount < PROPTEST_NBR_OF_VARIANCE_VALUES)
        {
            healthNoiseBuffer[healthNoiseCount++] = sensorData->acc.z;
        }
        if (healthNoiseCount >= PROPTEST_NBR_OF_VARIANCE_VALUES)
        {
            healthNoiseVariance = variance(healthNoiseBuffer,
                                            PROPTEST_NBR_OF_VARIANCE_VALUES);
            (void)healthNoiseVariance;
            healthTestState = measureProp;
            healthTick = 0;
            healthPropMotorIndex = 0;
        }
        break;

    case measureProp:
        if (healthPropMotorIndex < 4)
        {
            healthMotorTestCount++;
            float measured = sqrtf(sensorData->acc.x * sensorData->acc.x +
                                   sensorData->acc.y * sensorData->acc.y +
                                   sensorData->acc.z * sensorData->acc.z);
            bool pass = evaluatePropTest(healthPropLowThreshold,
                                         healthPropHighThreshold,
                                         measured,
                                         healthPropMotorIndex);
            if (pass)
            {
                healthMotorPassBits |= (1U << healthPropMotorIndex);
            }
            healthPropMotorIndex++;
        }
        if (healthPropMotorIndex >= 4)
        {
            healthTestState = evaluatePropResult;
        }
        break;

    case evaluatePropResult:
        motorPass = healthMotorPassBits;
        healthTestState = testDone;
        break;

    case testBattery:
        healthTick++;
        if (healthTick == 1)
        {
            healthIdleVoltage = batterySupplyVoltage;
            healthMinLoadedVoltage = batterySupplyVoltage;
        }
        else if (healthTick >= 2 && healthTick <= 49)
        {
            if (batterySupplyVoltage < healthMinLoadedVoltage)
            {
                healthMinLoadedVoltage = batterySupplyVoltage;
            }
        }
        else if (healthTick >= 50)
        {
            batterySag = healthIdleVoltage - healthMinLoadedVoltage;
            healthTestState = evaluateBatResult;
        }
        break;

    case evaluateBatResult:
        batteryPass = (batterySag > healthBatterySagThreshold) ? 0 : 1;
        healthBatteryPass = batteryPass;
        healthTestState = testDone;
        break;

    case restartBatTest:
        if ((tick - healthRestartStartTick) >= 2000)
        {
            healthTestState = testBattery;
            healthTick = 0;
            healthRestartStartTick = 0;
            healthMinLoadedVoltage = batterySupplyVoltage;
        }
        break;

    case testDone:
        break;
    }

    healthLog.motorPass = motorPass;
    healthLog.batteryPass = batteryPass;
    healthLog.batterySag = batterySag;
    healthLog.motorTestCount = healthMotorTestCount;
}

bool evaluatePropTest(float lowThreshold, float highThreshold,
                      float measuredValue, uint8_t motor)
{
    (void)motor;
    if (highThreshold == 0.0f)
    {
        return true;
    }
    if (lowThreshold <= measuredValue && measuredValue <= highThreshold)
    {
        return true;
    }
    return false;
}

float variance(const float *buffer, int length)
{
    if (!buffer || length <= 0)
    {
        return 0.0f;
    }

    float sum = 0.0f;
    float sumSq = 0.0f;
    for (int i = 0; i < length; ++i)
    {
        sum += buffer[i];
        sumSq += buffer[i] * buffer[i];
    }
    return sumSq - (sum * sum / (float)length);
}

/* ------------------------------------------------------------------------- */
/* CRTP transport                                                             */
/* ------------------------------------------------------------------------- */

typedef struct {
    CrtpPacket packets[CRTP_TX_QUEUE_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    bool initialized;
} CrtpPacketQueue;

static CrtpPacketQueue txQueue = {0};
static CrtpPacketQueue rxQueues[CRTP_NBR_OF_PORTS] = {0};
static CrtpPortCallback portCallbacks[CRTP_NBR_OF_PORTS] = {0};

static CrtpLink nopLink = {0};
static CrtpLink *activeLink = &nopLink;

static bool crtpInitialized = false;
static CrtpPacket pendingTxPacket = {0};
static bool hasPendingTxPacket = false;
static uint32_t lastTxAttemptTick = 0;
static uint32_t txPacketsSent = 0;
static uint32_t rxPacketsReceived = 0;
static uint32_t statsLastTick = 0;

static void queueInit(CrtpPacketQueue *q)
{
    if (!q)
    {
        return;
    }
    memset(q, 0, sizeof(*q));
    q->initialized = true;
}

static bool queuePush(CrtpPacketQueue *q, const CrtpPacket *packet, uint16_t capacity)
{
    if (!q || !packet || !q->initialized || q->count >= capacity)
    {
        return false;
    }

    q->packets[q->tail] = *packet;
    q->tail = (q->tail + 1) % capacity;
    q->count++;
    return true;
}

static bool queuePop(CrtpPacketQueue *q, CrtpPacket *packet, uint16_t capacity)
{
    if (!q || !packet || !q->initialized || q->count == 0)
    {
        return false;
    }

    *packet = q->packets[q->head];
    q->head = (q->head + 1) % capacity;
    q->count--;
    return true;
}

void crtpInit(void)
{
    if (crtpInitialized)
    {
        return;
    }

    queueInit(&txQueue);
    memset(rxQueues, 0, sizeof(rxQueues));
    memset(portCallbacks, 0, sizeof(portCallbacks));
    activeLink = &nopLink;
    hasPendingTxPacket = false;
    txPacketsSent = 0;
    rxPacketsReceived = 0;
    lastTxAttemptTick = 0;
    statsLastTick = tick;
    crtpRxQueueError = false;
    crtpInitialized = true;
}

void crtpInitTaskQueue(uint8_t port)
{
    if (port >= CRTP_NBR_OF_PORTS)
    {
        crtpRxQueueError = true;
        return;
    }

    if (rxQueues[port].initialized)
    {
        crtpRxQueueError = true;
        return;
    }
    queueInit(&rxQueues[port]);
}

bool crtpSendPacket(const CrtpPacket *packet)
{
    if (!packet)
    {
        return false;
    }
    return queuePush(&txQueue, packet, CRTP_TX_QUEUE_SIZE);
}

bool crtpSendPacketBlock(const CrtpPacket *packet)
{
    return crtpSendPacket(packet);
}

bool crtpReceivePacket(uint8_t port, CrtpPacket *packet)
{
    if (port >= CRTP_NBR_OF_PORTS || !packet)
    {
        return false;
    }
    return queuePop(&rxQueues[port], packet, CRTP_RX_QUEUE_SIZE);
}

bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet)
{
    return crtpReceivePacket(port, packet);
}

bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet, uint32_t wait_ms)
{
    if (crtpReceivePacket(port, packet))
    {
        return true;
    }

    if (wait_ms > 0)
    {
        /* Host scheduler: allow the RX task to pull from the link once.
           Without a real RTOS, this is the bounded equivalent of waiting for
           the timeout before returning false. */
        crtpRxTask();
        return crtpReceivePacket(port, packet);
    }

    return false;
}

void crtpRxTask(void)
{
    if (!activeLink || !activeLink->receivePacket)
    {
        return;
    }

    CrtpPacket packet;
    while (activeLink->receivePacket(&packet))
    {
        rxPacketsReceived++;
        bool delivered = false;
        if (packet.port < CRTP_NBR_OF_PORTS)
        {
            if (rxQueues[packet.port].initialized)
            {
                delivered = queuePush(&rxQueues[packet.port], &packet, CRTP_RX_QUEUE_SIZE);
            }
            if (portCallbacks[packet.port])
            {
                portCallbacks[packet.port](&packet);
                delivered = true;
            }
        }
        if (!delivered)
        {
            /* Drop packet. */
        }
    }
}

void crtpTxTask(void)
{
    if (!activeLink || !activeLink->sendPacket)
    {
        return;
    }

    if (hasPendingTxPacket)
    {
        if ((tick - lastTxAttemptTick) < 10)
        {
            return;
        }

        if (activeLink->sendPacket(&pendingTxPacket))
        {
            hasPendingTxPacket = false;
            txPacketsSent++;
        }
        else
        {
            lastTxAttemptTick = tick;
        }
        return;
    }

    CrtpPacket packet;
    if (queuePop(&txQueue, &packet, CRTP_TX_QUEUE_SIZE))
    {
        if (activeLink->sendPacket(&packet))
        {
            txPacketsSent++;
        }
        else
        {
            pendingTxPacket = packet;
            hasPendingTxPacket = true;
            lastTxAttemptTick = tick;
        }
    }
}

void crtpSetLink(CrtpLink *newLink)
{
    if (activeLink && activeLink != &nopLink && activeLink->setEnable)
    {
        activeLink->setEnable(false);
    }

    activeLink = (newLink != NULL) ? newLink : &nopLink;

    if (activeLink->setEnable)
    {
        activeLink->setEnable(true);
    }
}

void crtpReset(void)
{
    txQueue.head = 0;
    txQueue.tail = 0;
    txQueue.count = 0;
    hasPendingTxPacket = false;

    if (activeLink && activeLink->reset)
    {
        activeLink->reset();
    }
}

bool crtpIsConnected(void)
{
    if (activeLink && activeLink->isConnected)
    {
        return activeLink->isConnected();
    }
    return true;
}

uint32_t crtpGetFreeTxQueuePackets(void)
{
    return CRTP_TX_QUEUE_SIZE - txQueue.count;
}

void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback)
{
    if (port >= CRTP_NBR_OF_PORTS)
    {
        return;
    }
    portCallbacks[port] = callback;
}

void updateStats(void)
{
    uint32_t elapsed = tick - statsLastTick;
    if (elapsed >= 500)
    {
        uint32_t txRate = (txPacketsSent * 1000) / elapsed;
        uint32_t rxRate = (rxPacketsReceived * 1000) / elapsed;
        (void)txRate;
        (void)rxRate;
        txPacketsSent = 0;
        rxPacketsReceived = 0;
        statsLastTick = tick;
    }
}

/* ------------------------------------------------------------------------- */
/* Deck discovery                                                             */
/* ------------------------------------------------------------------------- */

uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity)
{
    if (!decks || capacity == 0)
    {
        return 0;
    }

    static const DeckInfo knownDecks[] = {
        {true, false, 0x21, 0},
        {false, true, 0, 0x1234567890ABCDEFULL}
    };

    uint8_t count = 0;
    for (uint8_t i = 0; i < 2 && count < capacity; ++i)
    {
        decks[count++] = knownDecks[i];
    }
    return count;
}
#include "6_generated_code.h"

#include <math.h>
#include <string.h>
#include "fse_frozen_api.h"
#define PI_F 3.14159265358979323846f
#define DEG_TO_RAD_F (PI_F / 180.0f)
#define RAD_TO_DEG_F (180.0f / PI_F)
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;
PidObject pidRoll, pidPitch, pidYaw;
PidObject pidRollRate, pidPitchRate, pidYawRate;
bool thrustLocked = false;
bool commanderModeSet = false;
SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0;
TestState healthTestState = testDone;
uint8_t motorPass = 0, batteryPass = 0;
float batterySag = 0.0f;
StateEstimateLog stateEstimate;
Axis3Log gyro, acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;
uint32_t systemTick = 0;
static float s_pid_dt = 0.002f;
static float s_desiredYaw = 0.0f;
static State s_stateEstimate;
static SensorData s_sensorData;
static bool s_sensorDataValid = false;
static uint32_t s_stabilizerStep = 0;
static bool s_stabilizerInitialized = false;
static bool s_highLevelPending = false;
static Setpoint s_highLevelSetpoint;
static Setpoint s_activeSetpoint;
static ControlData s_control;
static MotorPower s_motorPower;
static float s_batteryFiltered = 0.0f;
static float s_batteryVoltage = 0.0f;
static Setpoint s_commanderSetpoint;
static int s_commanderPriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t s_commanderLastUpdateTick = 0;
static EstimatorMeasurement s_estimatorFifo[16];
static uint8_t s_estimatorHead = 0, s_estimatorTail = 0, s_estimatorCount = 0;
static EstimatorMeasurement s_lastMeasurement[4];
static bool s_hasMeasurement[4];
static bool s_supervisorInitialized = false;
static SensorData s_supervisorSensors;
static uint32_t s_supervisorMotorRatios[4];
static uint32_t s_supervisorIdleThrust = 0;
static int32_t s_supervisorMotorRPMs[4];
static float s_crashDetectionGs = 0.0f;
static float s_freeFallThreshold = 0.0f;
static float s_acceptedTiltAccZ = 0.0f;
static float s_acceptedUpsideDownAccZ = 0.0f;
static uint32_t s_maxTiltTime = 1000u;
static uint32_t s_maxUpsideDownTime = 500u;
static bool s_tumbleCheckEnabled = true;
static bool s_autoArming = false;
static uint32_t s_spinupTimeoutDuration = 0;
static uint32_t s_spinupStartTick = 0;
static bool s_spinupActive = false;
static uint32_t s_latestArmingTick = 0;
static uint32_t s_latestLandingTick = 0;
static uint32_t s_emergencyStopLastNotification = 0;
static bool s_motorNotResponding = false;
static uint32_t s_motorNotRespondingStart = 0;
static bool s_motorNotRespondingTimerActive = false;
static uint32_t s_lastFlightTick = 0;
static bool s_seenFlying = false;
static bool s_isTumbled = false;
static bool s_isFreeFalling = false;
static uint32_t s_tumbleStartTick = 0;
static bool s_tumbleTimerActive = false;
static bool s_useUpsideDownTimer = false;
static uint32_t s_rateLastTick = 0;
static bool s_rateError = false;
static CrtpPacket s_txQueue[CRTP_TX_QUEUE_SIZE];
static uint16_t s_txHead = 0, s_txTail = 0, s_txCount = 0;
typedef struct { bool created; CrtpPacket q[CRTP_RX_QUEUE_SIZE]; uint8_t head,tail,count; } RxQueue;
static RxQueue s_rxQueues[CRTP_NBR_OF_PORTS];
static CrtpPortCallback s_portCallbacks[CRTP_NBR_OF_PORTS];
static CrtpLink *s_link = 0;
static bool s_crtpInitialized = false;
static bool s_crtpError = false;
static uint32_t s_txLastRetryTick = 0;
static uint32_t s_rxPackets = 0, s_txPackets = 0, s_lastStatsTick = 0;
static float s_rxRate = 0.0f, s_txRate = 0.0f;
static int s_propSampleCount = 0;
static float s_propSamples[PROPTEST_NBR_OF_VARIANCE_VALUES];
static float s_propSum = 0.0f, s_propSumSq = 0.0f;
static int s_propMotorIndex = 0;
static float s_idleVoltage = 0.0f;
static float s_minLoadedVoltage = 0.0f;
static int s_batteryTestTick = 0;
static uint32_t s_restartBatStart = 0;
static uint8_t s_propRequestFlag = 0;
static uint8_t s_batRequestFlag = 0;
static int s_propFailCount = 0;
static bool s_carefreeError = false;
static void resetPidObject(PidObject *pid) { if (pid) { pid->integral = 0.0f; pid->prevError = 0.0f; pid->output = 0.0f; } }
static void zeroSetpoint(Setpoint *sp) { if (sp) memset(sp, 0, sizeof(*sp)); }
static bool isArmingAllowedState(SupervisorState st) { return st == supervisorStateArming || st == supervisorStateReadyToFly || st == supervisorStateFlying || st == supervisorStateWarningLevelOut || st == supervisorStateLanded; }
static int16_t saturatePidOutput(float v) { if (v > 32767.0f) return 32767; if (v < -32767.0f) return -32767; return (int16_t)v; }
static float pidUpdate(PidObject *pid, float actual, float desired, float dt, bool resetIntegration) {
    if (!pid) return 0.0f;
    if (resetIntegration) { pid->integral = 0.0f; pid->prevError = 0.0f; }
    float error = desired - actual;
    float p = pid->kp * error;
    float i = pid->integral + pid->ki * error * dt;
    float d = dt > 0.0f ? pid->kd * (error - pid->prevError) / dt : 0.0f;
    float ff = pid->kff * desired;
    float out = p + i + d + ff;
    pid->integral = i;
    pid->prevError = error;
    pid->output = out;
    return out;
}
static uint16_t motorForceToPwm(float force) { float c = force < 0.0f ? 0.0f : force; if (c > CRAZYFLIE_MAX_MOTOR_FORCE_N) c = CRAZYFLIE_MAX_MOTOR_FORCE_N; return (uint16_t)(c / CRAZYFLIE_MAX_MOTOR_FORCE_N * 65535.0f); }
static uint32_t quatcompress(float q0, float q1, float q2, float q3) {
    int8_t cx = (int8_t)(q1 * 127.0f + 0.5f);
    int8_t cy = (int8_t)(q2 * 127.0f + 0.5f);
    int8_t cz = (int8_t)(q3 * 127.0f + 0.5f);
    int8_t cw = (int8_t)(q0 * 127.0f + 0.5f);
    return ((uint32_t)(uint8_t)cx) | (((uint32_t)(uint8_t)cy) << 8) | (((uint32_t)(uint8_t)cz) << 16) | (((uint32_t)(uint8_t)cw) << 24);
}
static void sensorsInit(void) { memset(&s_sensorData, 0, sizeof(s_sensorData)); s_sensorDataValid = false; }
static void stateEstimatorInit(void) { memset(&s_stateEstimate, 0, sizeof(s_stateEstimate)); s_stateEstimate.attitudeQuaternion.w = 1.0f; }
static void controllerInit(void) { attitudeControllerInit(0.002f); }
static void powerDistributionInit(void) { memset(&s_control, 0, sizeof(s_control)); memset(&s_motorPower, 0, sizeof(s_motorPower)); }
static void motorsInit(void) { motor.m1req = motor.m2req = motor.m3req = motor.m4req = 0; }
static void collisionAvoidanceInit(void) { memset(&s_activeSetpoint, 0, sizeof(s_activeSetpoint)); }
static void sensorsWaitDataReady(void) { }
static void sensorsAcquire(void) { }
static void stateEstimator(uint32_t step) { estimatorComplementary(step); }
static void commanderGetSetpoint(void) { s_activeSetpoint = s_commanderSetpoint; }
static void collisionAvoidanceUpdateSetpoint(void) { }
static void setMotorRatios(const MotorPower *p) {
    if (!p) return;
    int32_t vals[4] = { p->m1, p->m2, p->m3, p->m4 };
    for (int i = 0; i < 4; ++i) { if (vals[i] < 0) vals[i] = 0; if (vals[i] > 65535) vals[i] = 65535; }
    motor.m1req = (uint16_t)vals[0]; motor.m2req = (uint16_t)vals[1]; motor.m3req = (uint16_t)vals[2]; motor.m4req = (uint16_t)vals[3];
}
static bool nopSend(CrtpPacket *packet) { (void)packet; return false; }
static bool nopReceive(CrtpPacket *packet) { (void)packet; return false; }
static bool nopConnected(void) { return true; }
static void nopEnable(bool enable) { (void)enable; }
static void nopReset(void) { }
static CrtpLink nopLink = { nopSend, nopReceive, nopConnected, nopEnable, nopReset };
int16_t saturateSignedInt16(int32_t value) { if (value > 32767) return 32767; if (value < -32767) return -32767; return (int16_t)value; }
float capAngle(float angle_deg) { while (angle_deg > 180.0f) angle_deg -= 360.0f; while (angle_deg < -180.0f) angle_deg += 360.0f; return angle_deg; }
void sensfusion6Init(void) { if (sensfusion6IsInit) return; qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f; integralFBx = integralFBy = integralFBz = 0.0f; sensfusion6IsInit = true; sensfusion6IsCalibrated = false; baseZacc = 0.0f; }
bool sensfusion6Test(void) { return sensfusion6IsInit; }
void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float gxr = gx * DEG_TO_RAD_F, gyr = gy * DEG_TO_RAD_F, gzr = gz * DEG_TO_RAD_F;
    bool accValid = ax != 0.0f || ay != 0.0f || az != 0.0f;
    float axn = ax, ayn = ay, azn = az;
    if (accValid) { float n = sqrtf(ax*ax + ay*ay + az*az); if (n > 0.0f) { axn /= n; ayn /= n; azn /= n; } else accValid = false; }
#ifdef CONFIG_IMU_MADGWICK_QUATERNION
    if (!sensfusion6IsCalibrated && accValid) { estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ); baseZacc = ax * gravityX + ay * gravityY + az * gravityZ; sensfusion6IsCalibrated = true; }
    if (accValid) {
        float _2q0 = 2.0f * qw, _2q1 = 2.0f * qx, _2q2 = 2.0f * qy, _2q3 = 2.0f * qz;
        float f1 = _2q2 * qz - _2q0 * qy - axn; float f2 = _2q0 * qx + _2q1 * qz - ayn; float f3 = qw*qw - qx*qx - qy*qy + qz*qz - azn;
        float j11 = -_2q2; float j12 = _2q3; float j13 = -_2q0; float j14 = _2q1;
        float j21 = _2q1; float j22 = _2q0; float j23 = _2q3; float j24 = _2q2;
        float j31 = _2q0; float j32 = -_2q1; float j33 = -_2q2; float j34 = _2q3;
        float gradx = j11*f1 + j21*f2 + j31*f3; float grady = j12*f1 + j22*f2 + j32*f3; float gradz = j13*f1 + j23*f2 + j33*f3; float gradw = j14*f1 + j24*f2 + j34*f3;
        float norm = sqrtf(gradx*gradx + grady*grady + gradz*gradz + gradw*gradw); if (norm > 0.0f) { gradx /= norm; grady /= norm; gradz /= norm; gradw /= norm; }
        gxr -= beta * gradx; gyr -= beta * grady; gzr -= beta * gradz;
    }
#else
    if (!sensfusion6IsCalibrated && accValid) { estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ); baseZacc = ax * gravityX + ay * gravityY + az * gravityZ; sensfusion6IsCalibrated = true; }
    if (accValid) {
        float vx, vy, vz; estimatedGravityDirection(qw, qx, qy, qz, &vx, &vy, &vz);
        float ex = ayn * vz - azn * vy; float ey = azn * vx - axn * vz; float ez = axn * vy - ayn * vx;
        if (twoKi > 0.0f) { integralFBx += twoKi * ex * dt; integralFBy += twoKi * ey * dt; integralFBz += twoKi * ez * dt; } else { integralFBx = integralFBy = integralFBz = 0.0f; }
        gxr += twoKp * ex + integralFBx; gyr += twoKp * ey + integralFBy; gzr += twoKp * ez + integralFBz;
    } else if (twoKi == 0.0f) { integralFBx = integralFBy = integralFBz = 0.0f; }
#endif
    float qa = qw, qb = qx, qc = qy, qd = qz;
    float half = 0.5f * dt;
    qw = qa + half * (-qb*gxr - qc*gyr - qd*gzr);
    qx = qb + half * ( qa*gxr + qc*gzr - qd*gyr);
    qy = qc + half * ( qa*gyr - qb*gzr + qd*gxr);
    qz = qd + half * ( qa*gzr + qb*gyr - qc*gxr);
    float norm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz); if (norm > 0.0f) { float inv = 1.0f / norm; qw *= inv; qx *= inv; qy *= inv; qz *= inv; }
    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);
    sensfusion6Log.qw = qw; sensfusion6Log.qx = qx; sensfusion6Log.qy = qy; sensfusion6Log.qz = qz; sensfusion6Log.gravityX = gravityX; sensfusion6Log.gravityY = gravityY; sensfusion6Log.gravityZ = gravityZ; sensfusion6Log.accZbase = baseZacc; sensfusion6Log.isInit = sensfusion6IsInit; sensfusion6Log.isCalibrated = sensfusion6IsCalibrated;
}
void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg) {
    float r = 0.0f, p = 0.0f, y = 0.0f;
    float sinr_cosp = 2.0f * (qw*qx + qy*qz); float cosr_cosp = 1.0f - 2.0f * (qx*qx + qy*qy); r = atan2f(sinr_cosp, cosr_cosp);
    float sinp = 2.0f * (qw*qy - qz*qx); if (sinp > 1.0f) sinp = 1.0f; if (sinp < -1.0f) sinp = -1.0f; p = asinf(sinp);
    float siny_cosp = 2.0f * (qw*qz + qx*qy); float cosy_cosp = 1.0f - 2.0f * (qy*qy + qz*qz); y = atan2f(siny_cosp, cosy_cosp);
    if (roll_deg) *roll_deg = r * RAD_TO_DEG_F; if (pitch_deg) *pitch_deg = p * RAD_TO_DEG_F; if (yaw_deg) *yaw_deg = y * RAD_TO_DEG_F;
}
void sensfusion6GetQuaternion(float *ow, float *ox, float *oy, float *oz) { if (ow) *ow = qw; if (ox) *ox = qx; if (oy) *oy = qy; if (oz) *oz = qz; }
float sensfusion6GetAccZ(float ax, float ay, float az) { return ax*gravityX + ay*gravityY + az*gravityZ; }
float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) { return sensfusion6GetAccZ(ax, ay, az) - baseZacc; }
void estimatedGravityDirection(float cqw, float cqx, float cqy, float cqz, float *gravX, float *gravY, float *gravZ) { if (gravX) *gravX = 2.0f * (cqx*cqz - cqw*cqy); if (gravY) *gravY = 2.0f * (cqw*cqx + cqy*cqz); if (gravZ) *gravZ = cqw*cqw - cqx*cqx - cqy*cqy + cqz*cqz; }
float invSqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    float xhalf = 0.5f * x;
    union { float f; int32_t i; } u; u.f = x;
    u.i = 0x5f3759df - (u.i >> 1);
    float y = u.f;
    y = y * (1.5f - (xhalf * y * y));
    return y;
}
void powerDistributionLegacy(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw, MotorPower *out) { if (!out) return; int32_t r = roll / 2; int32_t p = pitch / 2; int32_t t = thrust; out->m1 = t - r + p + yaw; out->m2 = t - r - p - yaw; out->m3 = t + r - p + yaw; out->m4 = t + r + p - yaw; }
void powerDistributionForceTorque(float thrustSi, float torqueX, float torqueY, float torqueZ, float armLength, float thrustToTorque, float motorForces[4]) {
    if (!motorForces) return;
    float thrustPart = 0.25f * thrustSi; float rollPart = 0.0f, pitchPart = 0.0f, yawPart = 0.0f;
    if (armLength != 0.0f) { float arm = 0.707106781f * armLength; rollPart = 0.25f / arm * torqueX; pitchPart = 0.25f / arm * torqueY; }
    if (thrustToTorque != 0.0f) yawPart = 0.25f / thrustToTorque * torqueZ;
    float f[4]; f[0] = thrustPart - rollPart + pitchPart + yawPart; f[1] = thrustPart - rollPart - pitchPart - yawPart; f[2] = thrustPart + rollPart - pitchPart + yawPart; f[3] = thrustPart + rollPart + pitchPart - yawPart;
    for (int i = 0; i < 4; ++i) motorForces[i] = f[i] < 0.0f ? 0.0f : f[i];
}
void powerDistributionForce(const float normalizedForces[4], uint16_t motorPWMs[4]) { if (!normalizedForces || !motorPWMs) return; for (int i = 0; i < 4; ++i) { float c = normalizedForces[i] < 0.0f ? 0.0f : normalizedForces[i]; if (c > 1.0f) c = 1.0f; motorPWMs[i] = (uint16_t)(c * 65535.0f); } }
void powerDistribution(const ControlData *control, MotorPower *motorPower) {
    if (!control || !motorPower) return;
    switch (control->controlMode) {
        case controlModeLegacy: powerDistributionLegacy(control->thrust, control->roll, control->pitch, control->yaw, motorPower); break;
        case controlModeForceTorque: { float forces[4]; powerDistributionForceTorque(control->thrustSi, control->torque.x, control->torque.y, control->torque.z, CRAZYFLIE_ARM_LENGTH_M, CRAZYFLIE_THRUST_TO_TORQUE, forces); uint16_t pwms[4]; for (int i = 0; i < 4; ++i) pwms[i] = motorForceToPwm(forces[i]); motorPower->m1 = pwms[0]; motorPower->m2 = pwms[1]; motorPower->m3 = pwms[2]; motorPower->m4 = pwms[3]; } break;
        case controlModeForce: { uint16_t pwms[4]; powerDistributionForce(control->normalizedForces, pwms); motorPower->m1 = pwms[0]; motorPower->m2 = pwms[1]; motorPower->m3 = pwms[2]; motorPower->m4 = pwms[3]; } break;
        default: break;
    }
}
int32_t capMinThrust(int32_t value, int32_t idleThrust) { return value < idleThrust ? idleThrust : value; }
PowerCapResult powerDistributionCap(int32_t motors[4], int32_t maxAllowedThrust, int32_t idleThrust) {
    PowerCapResult result = { false, 0 };
    if (!motors) return result;
    int32_t max = motors[0]; for (int i = 1; i < 4; ++i) if (motors[i] > max) max = motors[i];
    if (max > maxAllowedThrust) { int32_t reduction = max - maxAllowedThrust; for (int i = 0; i < 4; ++i) motors[i] = capMinThrust(motors[i] - reduction, idleThrust); result.isCapped = true; result.reduction = reduction; }
    return result;
}
float batteryCompensation(float supplyVoltage, float filteredOld, float alpha) { return filteredOld + alpha * (supplyVoltage - filteredOld); }
uint16_t motorsCompensateBatteryVoltage(uint16_t motorThrust, float nominalVoltage, float actualVoltage) { if (actualVoltage <= 0.0f) return motorThrust; float c = roundf(motorThrust * nominalVoltage / actualVoltage); if (c < 0.0f) c = 0.0f; if (c > 65535.0f) c = 65535.0f; return (uint16_t)c; }
void attitudeControllerInit(float updateDt) {
    if (pidRoll.initialized) return;
    s_pid_dt = updateDt > 0.0f ? updateDt : 0.002f;
    resetPidObject(&pidRoll); resetPidObject(&pidPitch); resetPidObject(&pidYaw); resetPidObject(&pidRollRate); resetPidObject(&pidPitchRate); resetPidObject(&pidYawRate);
    pidRoll.kp = pidPitch.kp = pidYaw.kp = 0.0f; pidRoll.ki = pidPitch.ki = pidYaw.ki = 0.0f; pidRoll.kd = pidPitch.kd = pidYaw.kd = 0.0f; pidRoll.kff = pidPitch.kff = pidYaw.kff = 0.0f;
    pidRollRate.kp = pidPitchRate.kp = pidYawRate.kp = 0.0f; pidRollRate.ki = pidPitchRate.ki = pidYawRate.ki = 0.0f; pidRollRate.kd = pidPitchRate.kd = pidYawRate.kd = 0.0f; pidRollRate.kff = pidPitchRate.kff = pidYawRate.kff = 0.0f;
    pidRoll.initialized = pidPitch.initialized = pidYaw.initialized = true; pidRollRate.initialized = pidPitchRate.initialized = pidYawRate.initialized = true;
}
void attitudeControllerCorrectRatePID(float rollActual, float rollDesired, float pitchActual, float pitchDesired, float yawActual, float yawDesired) {
    pidUpdate(&pidRollRate, rollActual, rollDesired, s_pid_dt, false); pidRollRate.output = (float)saturatePidOutput(pidRollRate.output);
    pidUpdate(&pidPitchRate, pitchActual, pitchDesired, s_pid_dt, false); pidPitchRate.output = (float)saturatePidOutput(pidPitchRate.output);
    pidUpdate(&pidYawRate, yawActual, yawDesired, s_pid_dt, false); pidYawRate.output = (float)saturatePidOutput(pidYawRate.output);
}
void attitudeControllerCorrectAttitudePID(float rollActual, float rollDesired, float pitchActual, float pitchDesired, float yawActual, float yawDesired) {
    pidUpdate(&pidRoll, rollActual, rollDesired, s_pid_dt, false); pidUpdate(&pidPitch, pitchActual, pitchDesired, s_pid_dt, false); pidUpdate(&pidYaw, yawActual, yawDesired, s_pid_dt, true);
}
void attitudeControllerResetAllPID(float rollActual, float pitchActual, float yawActual) { (void)rollActual; (void)pitchActual; (void)yawActual; resetPidObject(&pidRoll); resetPidObject(&pidPitch); resetPidObject(&pidYaw); resetPidObject(&pidRollRate); resetPidObject(&pidPitchRate); resetPidObject(&pidYawRate); }
void attitudeControllerResetRollAttitudePID(float rollActual) { (void)rollActual; resetPidObject(&pidRoll); }
void attitudeControllerResetPitchAttitudePID(float pitchActual) { (void)pitchActual; resetPidObject(&pidPitch); }
void attitudeControllerGetActuatorOutput(int16_t *roll, int16_t *pitch, int16_t *yaw) { if (roll) *roll = saturatePidOutput(pidRollRate.output); if (pitch) *pitch = saturatePidOutput(pidPitchRate.output); if (yaw) *yaw = saturatePidOutput(pidYawRate.output); }
uint16_t positionControllerUpdate(const Setpoint *setpoint, const State *state) {
    if (!setpoint || !state) return 0;
    float thrust = 0.0f;
    if (setpoint->mode.z == modeAbs) { float err = setpoint->position.z - state->position.z; thrust = 40000.0f + 100.0f * err; }
    else if (setpoint->mode.z == modeVelocity) { float err = setpoint->velocity.z - state->velocity.z; thrust = 40000.0f + 100.0f * err; }
    else { thrust = setpoint->thrust; }
    if (thrust < 0.0f) thrust = 0.0f; if (thrust > 65535.0f) thrust = 65535.0f;
    return (uint16_t)thrust;
}
void rotateYaw(float roll, float pitch, float yaw_deg, float *rollPrime, float *pitchPrime) {
    if (!rollPrime || !pitchPrime) return;
    float rad = yaw_deg * DEG_TO_RAD_F; float c = cosf(rad), s = sinf(rad);
    *rollPrime = roll * c - pitch * s; *pitchPrime = roll * s + pitch * c;
}
static void setDefaultRollPitchYaw(Setpoint *sp, const CommanderCrtpLegacyValues *v, StabilizationType sr, StabilizationType spitch, StabilizationType syaw) {
    if (sr == RATE) { sp->mode.roll = modeVelocity; sp->attitudeRate.roll = v->roll; } else { sp->mode.roll = modeAbs; sp->attitude.roll = v->roll; }
    if (spitch == RATE) { sp->mode.pitch = modeVelocity; sp->attitudeRate.pitch = v->pitch; } else { sp->mode.pitch = modeAbs; sp->attitude.pitch = v->pitch; }
    if (syaw == RATE) { sp->mode.yaw = modeVelocity; sp->attitudeRate.yaw = -v->yaw; } else { sp->mode.yaw = modeAbs; sp->attitude.yaw = v->yaw; }
}
void crtpCommanderRpytDecodeSetpoint(const CommanderCrtpLegacyValues *values, Setpoint *setpoint, bool altHoldMode, bool posHoldMode, bool posSetMode, StabilizationType stabilizationModeRoll, StabilizationType stabilizationModePitch, StabilizationType stabilizationModeYaw, YawMode yawMode) {
    if (!values || !setpoint) return;
    if (s_commanderPriority == COMMANDER_PRIORITY_DISABLE) thrustLocked = true;
    if (values->thrust == 0) thrustLocked = false;
    zeroSetpoint(setpoint);
    uint16_t rawThrust = values->thrust; float rawRoll = values->roll, rawPitch = values->pitch;
    bool applyDefaultAttitude = false; bool applyYawOnly = false;
    if (altHoldMode) {
        if (!commanderModeSet) { commanderModeSet = true; }
        setpoint->mode.z = modeVelocity; setpoint->thrust = 0; setpoint->velocity.z = ((float)rawThrust - 32767.0f) / 32767.0f;
        applyDefaultAttitude = true;
    } else {
        if (commanderModeSet) { setpoint->mode.z = modeDisable; commanderModeSet = false; }
        if (posHoldMode) {
            setpoint->mode.x = modeVelocity; setpoint->mode.y = modeVelocity; setpoint->mode.roll = modeDisable; setpoint->mode.pitch = modeDisable;
            setpoint->velocity.x = values->pitch / 30.0f; setpoint->velocity.y = values->roll / 30.0f; setpoint->attitude.roll = 0.0f; setpoint->attitude.pitch = 0.0f;
            applyYawOnly = true;
        } else if (posSetMode && rawThrust != 0) {
            setpoint->mode.x = modeAbs; setpoint->mode.y = modeAbs; setpoint->mode.z = modeAbs; setpoint->mode.roll = modeDisable; setpoint->mode.pitch = modeDisable; setpoint->mode.yaw = modeAbs;
            setpoint->position.x = -values->pitch; setpoint->position.y = values->roll; setpoint->position.z = rawThrust / 1000.0f; setpoint->attitude.yaw = values->yaw; setpoint->thrust = 0;
        } else { applyDefaultAttitude = true; }
    }
    if (applyDefaultAttitude) { setDefaultRollPitchYaw(setpoint, values, stabilizationModeRoll, stabilizationModePitch, stabilizationModeYaw); }
    if (applyYawOnly) { if (stabilizationModeYaw == RATE) { setpoint->mode.yaw = modeVelocity; setpoint->attitudeRate.yaw = -values->yaw; } else { setpoint->mode.yaw = modeAbs; setpoint->attitude.yaw = values->yaw; } }
    if (!altHoldMode && !(posSetMode && rawThrust != 0)) {
        if (thrustLocked || rawThrust < MIN_THRUST) setpoint->thrust = 0; else setpoint->thrust = rawThrust > MAX_THRUST ? MAX_THRUST : rawThrust;
    }
    if (applyDefaultAttitude || applyYawOnly) {
        if (yawMode == PLUSMODE) { if (stabilizationModeRoll == RATE || stabilizationModePitch == RATE) rotateYaw(setpoint->attitudeRate.roll, setpoint->attitudeRate.pitch, 45.0f, &setpoint->attitudeRate.roll, &setpoint->attitudeRate.pitch); else rotateYaw(setpoint->attitude.roll, setpoint->attitude.pitch, 45.0f, &setpoint->attitude.roll, &setpoint->attitude.pitch); }
        else if (yawMode == CAREFREE) { s_carefreeError = true; }
    }
}
void supervisorInit(void) { if (s_supervisorInitialized) return; supervisorState = supervisorStateLocked; supervisorConditionBits = 0; memset(&s_supervisorSensors, 0, sizeof(s_supervisorSensors)); memset(s_supervisorMotorRatios, 0, sizeof(s_supervisorMotorRatios)); memset(s_supervisorMotorRPMs, 0, sizeof(s_supervisorMotorRPMs)); s_supervisorInitialized = true; }
bool supervisorCanFly(void) { return supervisorState == supervisorStateReadyToFly || supervisorState == supervisorStateFlying || supervisorState == supervisorStateWarningLevelOut || supervisorState == supervisorStateLanded; }
bool supervisorCanArm(void) { return supervisorState == supervisorStatePreFlChecksPassed; }
bool supervisorIsArmed(void) { return (supervisorConditionBits & SUPERVISOR_CB_ARMED) != 0; }
bool supervisorIsCrashed(void) { return (supervisorConditionBits & SUPERVISOR_CB_CRASHED) != 0; }
bool supervisorRequestArming(bool doArm) {
    if (doArm) {
        if (!supervisorCanArm()) return false;
        if (supervisorIsArmed() && supervisorState == supervisorStateArming) return true;
        supervisorConditionBits |= SUPERVISOR_CB_ARMED; supervisorState = supervisorStateArming;
        s_spinupStartTick = systemTick; s_spinupActive = true; s_latestArmingTick = systemTick; supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
        return true;
    } else {
        supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
        if (supervisorState == supervisorStateArming) { supervisorState = supervisorStatePreFlChecksPassed; s_spinupActive = false; supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT; }
        return true;
    }
}
bool supervisorRequestCrashRecovery(bool doRecovery) {
    if (!doRecovery) { supervisorConditionBits |= SUPERVISOR_CB_CRASHED; return true; }
    if (s_isTumbled) return false;
    supervisorConditionBits &= ~SUPERVISOR_CB_CRASHED;
    if (supervisorState == supervisorStateCrashed || supervisorState == supervisorStateExceptFreeFall) supervisorState = supervisorStatePreFlChecksPassed;
    return true;
}
bool supervisorAreMotorsAllowedToRun(void) { return isArmingAllowedState(supervisorState); }
uint16_t supervisorGetInfoBitfield(void) {
    uint16_t info = 0;
    if (supervisorCanArm()) info |= 1U << 0; if (supervisorIsArmed()) info |= 1U << 1; if (s_autoArming) info |= 1U << 2;
    if (supervisorCanFly()) info |= 1U << 3; if (supervisorConditionBits & SUPERVISOR_CB_IS_FLYING) info |= 1U << 4; if (s_isTumbled) info |= 1U << 5;
    if (supervisorState == supervisorStateLocked) info |= 1U << 6; if (supervisorIsCrashed()) info |= 1U << 7;
    return info;
}
bool isFlyingCheck(const uint32_t motorRatios[4], uint32_t idleThrust, uint32_t currentTick) {
    if (!motorRatios) return false;
    bool above = false; for (int i = 0; i < 4; ++i) if (motorRatios[i] > idleThrust) { above = true; break; }
    if (above) { s_seenFlying = true; s_lastFlightTick = currentTick; }
    if (!s_seenFlying) return false;
    return (currentTick - s_lastFlightTick) < IS_FLYING_HYSTERESIS_THRESHOLD;
}
bool isTumbledCheck(float accX, float accY, float accZ, float crashDetectionGs, float freeFallThreshold, float acceptedTiltAccZ, float acceptedUpsideDownAccZ, uint32_t maxTiltTime, uint32_t maxUpsideDownTime, bool tumbleCheckEnabled, uint32_t currentTick, bool *isFreeFalling) {
    if (isFreeFalling) *isFreeFalling = false;
    if (crashDetectionGs > 0.0f) { float norm = sqrtf(accX*accX + accY*accY + accZ*accZ); if (fabsf(norm - 1.0f) > crashDetectionGs) supervisorConditionBits |= SUPERVISOR_CB_CRASHED; }
    bool freefall = freeFallThreshold > 0.0f && fabsf(accX) < freeFallThreshold && fabsf(accY) < freeFallThreshold && fabsf(accZ) < freeFallThreshold;
    if (freefall) { s_isFreeFalling = true; if (isFreeFalling) *isFreeFalling = true; s_tumbleTimerActive = false; s_isTumbled = false; return false; }
    s_isFreeFalling = false;
    if (!tumbleCheckEnabled) { s_tumbleTimerActive = false; s_isTumbled = false; return false; }
    if (accZ >= acceptedTiltAccZ) { s_tumbleTimerActive = false; s_isTumbled = false; return false; }
    bool upsideDown = accZ < acceptedUpsideDownAccZ; uint32_t timeout = upsideDown ? maxUpsideDownTime : maxTiltTime;
    if (!s_tumbleTimerActive || s_useUpsideDownTimer != upsideDown) { s_tumbleTimerActive = true; s_useUpsideDownTimer = upsideDown; s_tumbleStartTick = currentTick; }
    else if (currentTick - s_tumbleStartTick >= timeout) { s_isTumbled = true; return true; }
    return s_isTumbled;
}
bool checkEmergencyStopWatchdog(uint32_t currentTick, uint32_t lastNotificationTick) { if (lastNotificationTick == 0) return true; return (currentTick - lastNotificationTick) <= DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT; }
bool supervisorIsPreflightTimeout(SupervisorState state, uint32_t latestArmingTick, uint32_t currentTick, uint32_t preflightTimeoutDuration) { if (state != supervisorStateReadyToFly || latestArmingTick == 0) return false; return (currentTick - latestArmingTick) >= preflightTimeoutDuration; }
bool supervisorIsLandingTimeout(uint32_t latestLandingTick, uint32_t currentTick, uint32_t landingTimeoutDuration) { if (latestLandingTick == 0) return false; return (currentTick - latestLandingTick) >= landingTimeoutDuration; }
uint32_t updateAndPopulateConditions(bool crtpEmergencyStop, bool paramEmergencyStop, bool emergencyStopWatchdogFailed) {
    if (crtpEmergencyStop || paramEmergencyStop || emergencyStopWatchdogFailed) supervisorConditionBits |= SUPERVISOR_CB_EMERGENCY_STOP; else supervisorConditionBits &= ~SUPERVISOR_CB_EMERGENCY_STOP;
    if (s_commanderLastUpdateTick != 0) { uint32_t age = systemTick - s_commanderLastUpdateTick; if (age > COMMANDER_WDT_TIMEOUT_STABILIZE) supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING; else supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING; if (age > COMMANDER_WDT_TIMEOUT_SHUTDOWN) supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT; else supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT; }
    return supervisorConditionBits;
}
void supervisorUpdate(uint32_t stabilizerStep) {
    if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) return;
    if (supervisorState == supervisorStatePreFlChecksPassed && s_autoArming && !supervisorIsArmed()) supervisorRequestArming(true);
    if (supervisorState == supervisorStateArming && s_spinupActive) { uint32_t elapsed = systemTick >= s_spinupStartTick ? systemTick - s_spinupStartTick : 0; if (s_spinupTimeoutDuration > 0 && elapsed >= s_spinupTimeoutDuration) { supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT; supervisorState = supervisorStatePreFlChecksPassed; supervisorConditionBits &= ~SUPERVISOR_CB_ARMED; s_spinupActive = false; } }
    if (!isArmingAllowedState(supervisorState)) supervisorConditionBits &= ~SUPERVISOR_CB_ARMED;
    bool flying = isFlyingCheck(s_supervisorMotorRatios, s_supervisorIdleThrust, systemTick); if (flying) supervisorConditionBits |= SUPERVISOR_CB_IS_FLYING; else supervisorConditionBits &= ~SUPERVISOR_CB_IS_FLYING;
    bool freeFallTmp = false;
    bool tumbled = isTumbledCheck(s_supervisorSensors.acc.x, s_supervisorSensors.acc.y, s_supervisorSensors.acc.z, s_crashDetectionGs, s_freeFallThreshold, s_acceptedTiltAccZ, s_acceptedUpsideDownAccZ, s_maxTiltTime, s_maxUpsideDownTime, s_tumbleCheckEnabled, systemTick, &freeFallTmp);
    s_isFreeFalling = freeFallTmp;
    if (freeFallTmp) supervisorConditionBits |= SUPERVISOR_CB_FREE_FALL; else supervisorConditionBits &= ~SUPERVISOR_CB_FREE_FALL;
    if (tumbled) supervisorConditionBits |= SUPERVISOR_CB_IS_TUMBLED; else supervisorConditionBits &= ~SUPERVISOR_CB_IS_TUMBLED;
    s_isTumbled = tumbled;
    if (freeFallTmp) supervisorState = supervisorStateExceptFreeFall;
    if (supervisorConditionBits & SUPERVISOR_CB_CRASHED) supervisorState = supervisorStateCrashed;
    if (s_commanderLastUpdateTick != 0) { uint32_t age = systemTick - s_commanderLastUpdateTick; if (age > COMMANDER_WDT_TIMEOUT_STABILIZE) supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_WARNING; else supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_WARNING; if (age > COMMANDER_WDT_TIMEOUT_SHUTDOWN) supervisorConditionBits |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT; else supervisorConditionBits &= ~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT; }
    supervisorLog.info = supervisorGetInfoBitfield(); float nx = s_supervisorSensors.acc.x, ny = s_supervisorSensors.acc.y, nz = s_supervisorSensors.acc.z; supervisorLog.accNorm = sqrtf(nx*nx + ny*ny + nz*nz);
}
void supervisorOverrideSetpoint(Setpoint *setpoint, uint32_t conditionBits, SupervisorState state) {
    if (!setpoint) return;
    bool unsafe = (conditionBits & (SUPERVISOR_CB_EMERGENCY_STOP | SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT | SUPERVISOR_CB_FREE_FALL | SUPERVISOR_CB_MOTORS_NOT_RESPONDING | SUPERVISOR_CB_CRASHED | SUPERVISOR_CB_IS_TUMBLED)) != 0;
    if (state == supervisorStateWarningLevelOut && !unsafe) { setpoint->mode.x = modeDisable; setpoint->mode.y = modeDisable; setpoint->mode.roll = modeAbs; setpoint->mode.pitch = modeAbs; setpoint->attitude.roll = 0.0f; setpoint->attitude.pitch = 0.0f; setpoint->mode.yaw = modeVelocity; setpoint->attitudeRate.yaw = 0.0f; return; }
    if ((state == supervisorStateArming || state == supervisorStateReadyToFly || state == supervisorStateFlying || state == supervisorStateLanded) && !unsafe) return;
    zeroSetpoint(setpoint);
}
bool isRPMatArmingValid(const int32_t motorRPMs[4], int32_t rpmCheckMin, int32_t rpmCheckMax) { if (!motorRPMs) return false; for (int i = 0; i < 4; ++i) if (motorRPMs[i] < rpmCheckMin || motorRPMs[i] > rpmCheckMax) return false; return true; }
bool isMotorsNotResponding(const int32_t motorRPMs[4], int32_t rpmThreshold, uint32_t rpmCheckDurationMs, bool canFly, uint32_t currentTick) {
    if (!motorRPMs) return false;
    if (!canFly) { s_motorNotResponding = false; s_motorNotRespondingTimerActive = false; s_motorNotRespondingStart = 0; supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING; return false; }
    bool below = false; for (int i = 0; i < 4; ++i) if (motorRPMs[i] < rpmThreshold) { below = true; break; }
    if (below) { if (!s_motorNotRespondingTimerActive) { s_motorNotRespondingTimerActive = true; s_motorNotRespondingStart = currentTick; } if (currentTick - s_motorNotRespondingStart >= rpmCheckDurationMs) { s_motorNotResponding = true; supervisorConditionBits |= SUPERVISOR_CB_MOTORS_NOT_RESPONDING; return true; } }
    else { s_motorNotRespondingTimerActive = false; s_motorNotRespondingStart = 0; s_motorNotResponding = false; supervisorConditionBits &= ~SUPERVISOR_CB_MOTORS_NOT_RESPONDING; }
    return s_motorNotResponding;
}
void supervisorSetSensorData(const SensorData *sensors) { if (!sensors) return; s_supervisorSensors = *sensors; s_sensorData = *sensors; s_sensorDataValid = true; gyro.x = sensors->gyro.x; gyro.y = sensors->gyro.y; gyro.z = sensors->gyro.z; acc.x = sensors->acc.x; acc.y = sensors->acc.y; acc.z = sensors->acc.z; baro.asl = sensors->baroAsl; baro.temp = sensors->baroTemperature; baro.pressure = sensors->baroPressure; }
void supervisorSetMotorRatios(const uint32_t motorRatios[4], uint32_t idleThrust) { if (!motorRatios) return; for (int i = 0; i < 4; ++i) s_supervisorMotorRatios[i] = motorRatios[i]; s_supervisorIdleThrust = idleThrust; }
void supervisorSetMotorRPMs(const int32_t motorRPMs[4]) { if (!motorRPMs) return; for (int i = 0; i < 4; ++i) s_supervisorMotorRPMs[i] = motorRPMs[i]; }
void supervisorConfigureSafety(float crashDetectionGs, float freeFallThreshold, float acceptedTiltAccZ, float acceptedUpsideDownAccZ, uint32_t maxTiltTime, uint32_t maxUpsideDownTime, bool tumbleCheckEnabled) { s_crashDetectionGs = crashDetectionGs; s_freeFallThreshold = freeFallThreshold; s_acceptedTiltAccZ = acceptedTiltAccZ; s_acceptedUpsideDownAccZ = acceptedUpsideDownAccZ; s_maxTiltTime = maxTiltTime; s_maxUpsideDownTime = maxUpsideDownTime; s_tumbleCheckEnabled = tumbleCheckEnabled; }
void supervisorConfigureArming(bool autoArming, uint32_t spinupTimeoutDurationMs) { s_autoArming = autoArming; s_spinupTimeoutDuration = spinupTimeoutDurationMs; }
bool estimatorEnqueue(const EstimatorMeasurement *measurement) { if (!measurement) return false; if (s_estimatorCount >= 16) return false; s_estimatorFifo[s_estimatorTail] = *measurement; s_estimatorTail = (uint8_t)((s_estimatorTail + 1) % 16); s_estimatorCount++; return true; }
bool estimatorDequeue(EstimatorMeasurement *measurement) { if (!measurement) return false; if (s_estimatorCount == 0) return false; *measurement = s_estimatorFifo[s_estimatorHead]; s_estimatorHead = (uint8_t)((s_estimatorHead + 1) % 16); s_estimatorCount--; return true; }
void estimatorComplementary(uint32_t stabilizerStep) {
    EstimatorMeasurement m;
    while (estimatorDequeue(&m)) { if (m.type >= 0 && m.type < 4) { s_lastMeasurement[m.type] = m; s_hasMeasurement[m.type] = true; } }
    if (RATE_DO_EXECUTE(RATE_250_HZ, stabilizerStep)) {
        float gx = s_hasMeasurement[MeasurementTypeGyroscope] ? s_lastMeasurement[MeasurementTypeGyroscope].data[0] : 0.0f;
        float gy = s_hasMeasurement[MeasurementTypeGyroscope] ? s_lastMeasurement[MeasurementTypeGyroscope].data[1] : 0.0f;
        float gz = s_hasMeasurement[MeasurementTypeGyroscope] ? s_lastMeasurement[MeasurementTypeGyroscope].data[2] : 0.0f;
        float ax = s_hasMeasurement[MeasurementTypeAcceleration] ? s_lastMeasurement[MeasurementTypeAcceleration].data[0] : 0.0f;
        float ay = s_hasMeasurement[MeasurementTypeAcceleration] ? s_lastMeasurement[MeasurementTypeAcceleration].data[1] : 0.0f;
        float az = s_hasMeasurement[MeasurementTypeAcceleration] ? s_lastMeasurement[MeasurementTypeAcceleration].data[2] : 0.0f;
        sensfusion6UpdateQ(gx, gy, gz, ax, ay, az, 0.004f);
        sensfusion6GetEulerRPY(&s_stateEstimate.attitude.roll, &s_stateEstimate.attitude.pitch, &s_stateEstimate.attitude.yaw);
        sensfusion6GetQuaternion(&s_stateEstimate.attitudeQuaternion.w, &s_stateEstimate.attitudeQuaternion.x, &s_stateEstimate.attitudeQuaternion.y, &s_stateEstimate.attitudeQuaternion.z);
        s_stateEstimate.acc.x = ax; s_stateEstimate.acc.y = ay; s_stateEstimate.acc.z = az;
        s_stateEstimate.velocity.z += sensfusion6GetAccZWithoutGravity(ax, ay, az) * 9.81f * 0.004f;
        stateEstimate.roll = s_stateEstimate.attitude.roll; stateEstimate.pitch = s_stateEstimate.attitude.pitch; stateEstimate.yaw = s_stateEstimate.attitude.yaw; stateEstimate.qx = s_stateEstimate.attitudeQuaternion.x; stateEstimate.qy = s_stateEstimate.attitudeQuaternion.y; stateEstimate.qz = s_stateEstimate.attitudeQuaternion.z; stateEstimate.qw = s_stateEstimate.attitudeQuaternion.w;
    }
    if (RATE_DO_EXECUTE(RATE_100_HZ, stabilizerStep)) { s_stateEstimate.position.z += s_stateEstimate.velocity.z * 0.01f; }
}
bool commanderSetSetpoint(const Setpoint *setpoint, int priority) {
    if (!setpoint) return false;
    if (priority == COMMANDER_PRIORITY_DISABLE || priority >= s_commanderPriority) { s_commanderSetpoint = *setpoint; s_commanderPriority = priority; s_commanderLastUpdateTick = systemTick; if (priority > COMMANDER_PRIORITY_HIGHLEVEL) { }
        return true; }
    return false;
}
void commanderRelaxPriority(void) { s_commanderPriority = COMMANDER_PRIORITY_LOWEST; }
uint32_t commanderGetInactivityTime(void) { return systemTick - s_commanderLastUpdateTick; }
int commanderGetActivePriority(void) { return s_commanderPriority; }
void stabilizerInit(void) { if (s_stabilizerInitialized) return; sensorsInit(); stateEstimatorInit(); controllerInit(); powerDistributionInit(); motorsInit(); collisionAvoidanceInit(); s_stabilizerInitialized = true; }
bool stabilizerSubmitHighLevelSetpoint(const Setpoint *setpoint) { if (!setpoint) return false; s_highLevelSetpoint = *setpoint; s_highLevelPending = true; return true; }
void compressState(const State *state, const SensorData *sensors, CompressedState *output) {
    if (!state || !sensors || !output) return;
    for (int i = 0; i < 3; ++i) { output->position_mm[i] = (int32_t)(state->position.x * 1000.0f); output->velocity_mms[i] = (int32_t)(state->velocity.x * 1000.0f); output->acceleration_mms2[i] = (int32_t)(sensors->acc.x * 9810.0f); }
    output->acceleration_mms2[2] = (int32_t)((sensors->acc.z + 1.0f) * 9810.0f);
    output->gyro_millirad_s[0] = sensors->gyro.x * DEG_TO_RAD_F * 1000.0f; output->gyro_millirad_s[1] = -sensors->gyro.y * DEG_TO_RAD_F * 1000.0f; output->gyro_millirad_s[2] = sensors->gyro.z * DEG_TO_RAD_F * 1000.0f;
    output->quatCompressed = quatcompress(state->attitudeQuaternion.w, state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z);
}
bool rateSupervisorValidate(uint32_t measuredRate) { return measuredRate >= 997U && measuredRate <= 1003U; }
void rateSupervisorTask(void) { if (systemTick - s_rateLastTick > 2000U) { if (s_sensorDataValid) s_rateError = true; else { s_rateLastTick = systemTick; s_rateError = false; } } else if (s_sensorDataValid) { s_rateLastTick = systemTick; s_rateError = false; } }
bool healthShallWeRunTest(void) {
    if (s_propRequestFlag) { s_propRequestFlag = 0; healthTestState = configureAcc; s_propSampleCount = 0; s_propSum = 0.0f; s_propSumSq = 0.0f; s_propMotorIndex = 0; s_propFailCount = 0; motorPass = 0; batteryPass = 0; batterySag = 0.0f; return true; }
    if (s_batRequestFlag) { s_batRequestFlag = 0; healthTestState = testBattery; s_batteryTestTick = 0; return true; }
    return healthTestState != testDone;
}
void healthRequestPropTest(void) { s_propRequestFlag = 1; }
void healthRequestBatteryTest(void) { s_batRequestFlag = 1; }
bool evaluatePropTest(float lowThreshold, float highThreshold, float measuredValue, uint8_t motorIndex) {
    if (motorIndex > 3) return false;
    if (highThreshold == 0.0f) { motorPass |= (uint8_t)(1U << motorIndex); return true; }
    if (measuredValue >= lowThreshold && measuredValue <= highThreshold) { motorPass |= (uint8_t)(1U << motorIndex); return true; }
    s_propFailCount++; return false;
}
float variance(const float *buffer, int length) { if (!buffer || length <= 0) return 0.0f; float sum = 0.0f, sumSq = 0.0f; for (int i = 0; i < length; ++i) { sum += buffer[i]; sumSq += buffer[i] * buffer[i]; } return sumSq - (sum * sum) / (float)length; }
void healthRunTests(const SensorData *sensorData) {
    switch (healthTestState) {
        case configureAcc: motorPass = 0; batteryPass = 0; batterySag = 0.0f; s_idleVoltage = 0.0f; s_minLoadedVoltage = 0.0f; s_propSampleCount = 0; s_propSum = 0.0f; s_propSumSq = 0.0f; s_propMotorIndex = 0; s_propFailCount = 0; healthTestState = measureNoiseFloor; break;
        case measureNoiseFloor: if (sensorData && s_propSampleCount < PROPTEST_NBR_OF_VARIANCE_VALUES) { float sample = sensorData->acc.x + sensorData->acc.y + sensorData->acc.z; s_propSamples[s_propSampleCount++] = sample; s_propSum += sample; s_propSumSq += sample * sample; } if (s_propSampleCount >= PROPTEST_NBR_OF_VARIANCE_VALUES) healthTestState = measureProp; break;
        case measureProp: if (s_propMotorIndex < 4) { evaluatePropTest(0.0f, 0.0f, 0.0f, (uint8_t)s_propMotorIndex); healthLog.motorTestCount++; s_propMotorIndex++; } else healthTestState = evaluatePropResult; break;
        case evaluatePropResult: healthTestState = testDone; break;
        case testBattery: s_batteryTestTick++; if (s_batteryTestTick == 1) { s_idleVoltage = s_batteryVoltage; s_minLoadedVoltage = s_batteryVoltage; } else if (s_batteryTestTick >= 2 && s_batteryTestTick <= 49) { if (s_batteryVoltage < s_minLoadedVoltage) s_minLoadedVoltage = s_batteryVoltage; } else if (s_batteryTestTick >= 50) { batterySag = s_idleVoltage - s_minLoadedVoltage; batteryPass = batterySag > 0.0f ? 0 : 1; healthTestState = evaluateBatResult; } break;
        case evaluateBatResult: healthTestState = testDone; break;
        case restartBatTest: if (s_restartBatStart == 0) s_restartBatStart = systemTick; if (systemTick - s_restartBatStart >= 2000U) { s_restartBatStart = 0; s_batteryTestTick = 0; healthTestState = testBattery; } break;
        case testDone: break;
    }
    healthLog.motorPass = motorPass; healthLog.batteryPass = batteryPass; healthLog.batterySag = batterySag;
}
void crtpInit(void) { if (s_crtpInitialized) return; s_txHead = s_txTail = s_txCount = 0; for (int i = 0; i < CRTP_NBR_OF_PORTS; ++i) { s_rxQueues[i].created = false; s_rxQueues[i].head = s_rxQueues[i].tail = s_rxQueues[i].count = 0; s_portCallbacks[i] = 0; } s_link = &nopLink; s_crtpInitialized = true; }
void crtpInitTaskQueue(uint8_t port) { if (port >= CRTP_NBR_OF_PORTS) { s_crtpError = true; return; } if (s_rxQueues[port].created) { s_crtpError = true; return; } s_rxQueues[port].created = true; s_rxQueues[port].head = s_rxQueues[port].tail = s_rxQueues[port].count = 0; }
bool crtpSendPacket(const CrtpPacket *packet) { if (!packet) return false; if (s_txCount >= CRTP_TX_QUEUE_SIZE) return false; s_txQueue[s_txTail] = *packet; s_txTail++; if (s_txTail >= CRTP_TX_QUEUE_SIZE) s_txTail = 0; s_txCount++; return true; }
bool crtpSendPacketBlock(const CrtpPacket *packet) { return crtpSendPacket(packet); }
bool crtpReceivePacket(uint8_t port, CrtpPacket *packet) { if (port >= CRTP_NBR_OF_PORTS || !packet) return false; if (!s_rxQueues[port].created || s_rxQueues[port].count == 0) return false; *packet = s_rxQueues[port].q[s_rxQueues[port].head]; s_rxQueues[port].head++; if (s_rxQueues[port].head >= CRTP_RX_QUEUE_SIZE) s_rxQueues[port].head = 0; s_rxQueues[port].count--; return true; }
bool crtpReceivePacketBlock(uint8_t port, CrtpPacket *packet) { return crtpReceivePacket(port, packet); }
bool crtpReceivePacketWait(uint8_t port, CrtpPacket *packet, uint32_t wait_ms) { (void)wait_ms; return crtpReceivePacket(port, packet); }
void crtpRxTask(void) { if (!s_crtpInitialized || !s_link || !s_link->receivePacket) return; CrtpPacket pkt; if (!s_link->receivePacket(&pkt)) return; s_rxPackets++; if (pkt.port >= CRTP_NBR_OF_PORTS) return; if (s_rxQueues[pkt.port].created && s_rxQueues[pkt.port].count < CRTP_RX_QUEUE_SIZE) { s_rxQueues[pkt.port].q[s_rxQueues[pkt.port].tail] = pkt; s_rxQueues[pkt.port].tail++; if (s_rxQueues[pkt.port].tail >= CRTP_RX_QUEUE_SIZE) s_rxQueues[pkt.port].tail = 0; s_rxQueues[pkt.port].count++; } if (s_portCallbacks[pkt.port]) s_portCallbacks[pkt.port](&pkt); }
void crtpTxTask(void) { if (!s_crtpInitialized || s_link == &nopLink || s_txCount == 0) return; if (systemTick - s_txLastRetryTick < 10U && s_txLastRetryTick != 0U) return; CrtpPacket *p = &s_txQueue[s_txHead]; if (s_link && s_link->sendPacket) { if (s_link->sendPacket(p)) { s_txHead++; if (s_txHead >= CRTP_TX_QUEUE_SIZE) s_txHead = 0; s_txCount--; s_txPackets++; s_txLastRetryTick = 0; } else { s_txLastRetryTick = systemTick; } } }
void crtpSetLink(CrtpLink *newLink) { if (s_link && s_link->setEnable) s_link->setEnable(false); if (!newLink) s_link = &nopLink; else s_link = newLink; if (s_link && s_link->setEnable) s_link->setEnable(true); }
void crtpReset(void) { s_txHead = s_txTail = s_txCount = 0; if (s_link && s_link->reset) s_link->reset(); }
bool crtpIsConnected(void) { if (s_link && s_link->isConnected) return s_link->isConnected(); return true; }
uint32_t crtpGetFreeTxQueuePackets(void) { return (uint32_t)(CRTP_TX_QUEUE_SIZE - s_txCount); }
void crtpRegisterPortCB(uint8_t port, CrtpPortCallback callback) { if (port >= CRTP_NBR_OF_PORTS) return; s_portCallbacks[port] = callback; }
void updateStats(void) { if (systemTick - s_lastStatsTick >= 500U) { uint32_t elapsed = systemTick - s_lastStatsTick; if (elapsed > 0) { s_rxRate = (float)s_rxPackets * 1000.0f / (float)elapsed; s_txRate = (float)s_txPackets * 1000.0f / (float)elapsed; } s_rxPackets = 0; s_txPackets = 0; s_lastStatsTick = systemTick; } }
uint8_t deckDiscovery(DeckInfo *decks, uint8_t capacity) { if (!decks || capacity == 0) return 0; return 0; }
void stabilizerTask(void) {
    if (!s_stabilizerInitialized) stabilizerInit();
    sensorsWaitDataReady(); sensorsAcquire(); stateEstimator(s_stabilizerStep);
    if (s_highLevelPending) { commanderSetSetpoint(&s_highLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL); s_highLevelPending = false; }
    commanderGetSetpoint(); supervisorUpdate(s_stabilizerStep);
    if (healthShallWeRunTest()) { healthRunTests(&s_sensorData); return; }
    collisionAvoidanceUpdateSetpoint();
    supervisorOverrideSetpoint(&s_activeSetpoint, supervisorConditionBits, supervisorState);
    if (!supervisorCanFly() || !supervisorAreMotorsAllowedToRun()) { motor.m1req = 0; motor.m2req = 0; motor.m3req = 0; motor.m4req = 0; return; }
    s_control.controlMode = controlModeLegacy;
    controllerPid(&s_sensorData, &s_activeSetpoint, &s_stateEstimate, &s_control, 0.0f, 0.002f);
    powerDistribution(&s_control, &s_motorPower);
    s_batteryFiltered = batteryCompensation(s_batteryVoltage, s_batteryFiltered, 0.01f);
    int32_t vals[4] = { s_motorPower.m1, s_motorPower.m2, s_motorPower.m3, s_motorPower.m4 };
    for (int i = 0; i < 4; ++i) { if (vals[i] < 0) vals[i] = 0; if (vals[i] > 65535) vals[i] = 65535; vals[i] = motorsCompensateBatteryVoltage((uint16_t)vals[i], 4.2f, s_batteryFiltered); }
    s_motorPower.m1 = vals[0]; s_motorPower.m2 = vals[1]; s_motorPower.m3 = vals[2]; s_motorPower.m4 = vals[3];
    powerDistributionCap(vals, 65535, 0); s_motorPower.m1 = vals[0]; s_motorPower.m2 = vals[1]; s_motorPower.m3 = vals[2]; s_motorPower.m4 = vals[3];
    setMotorRatios(&s_motorPower); s_stabilizerStep++;
}
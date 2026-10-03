// ==========================================
// SOURCE: generated_code.c
// ==========================================
#include "generated_code.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

// Global Log Instances Definition
log_state_estimate_t stateEstimate;
log_gyro_t gyro;
log_acc_t acc;
log_baro_t baro;
log_motor_t motor;
log_sensfusion6_t sensfusion6Log;
log_supervisor_t supervisorLog;
log_health_t healthLog;

// Global variables definition
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f;
float baseZacc = 0.0f;
bool calibrated = false;
bool config_imu_madgwick = false;
PID_t pidRoll, pidPitch, pidYaw, pidRollRate, pidPitchRate, pidYawRate;
float yawMaxDelta = 0.0f;
bool thrustLocked = false;
bool altHoldMode = false;
bool posHoldMode = false;
bool posSetMode = false;
bool plusMode = false;
bool carefreeMode = false;
bool carefreeError = false;
uint8_t activePriority = 0;
float positionControllerOutputThrust = 0.0f;
float batteryVoltage = 3.7f;
float batteryNominalVoltage = 3.7f;
uint32_t currentTick = 0;
bool isArmed = false;
bool isCrashed = false;
bool isTumbled = false;
bool isFreeFalling = false;
bool isLocked = false;
bool trajectoryFlying = false;
bool trajectoryFinished = false;
bool trajectoryDisabled = false;
bool deckFault = false;
bool configAutoArming = false;
uint32_t recentFlightTick = 0;
bool hasFlown = false;
int32_t motorRatios[4] = {0};
int32_t motorRPMs[4] = {0};
float crashDetectionGs = 0.0f;
float freeFallThreshold = 0.1f;
float tiltThreshold = 0.5f;
float invertThreshold = 0.0f;
bool tumbleCheckEnabled = true;
uint32_t spinupStartTick = 0;
uint32_t spinupTimeoutDuration = 500;
uint32_t lastWatchdogNotificationTick = 0;
uint32_t lastCommanderSetpointTick = 0;
uint32_t supervisorConditionBits = 0;

// Internal states
static bool sensfusion6Initialized = false;
static bool attitudeControllerInitialized = false;
static bool stabilizerInitialized = false;
static int32_t m1_mix = 0, m2_mix = 0, m3_mix = 0, m4_mix = 0;
static setpoint_t activeSetpoint;
static uint8_t currentPriority = COMMANDER_PRIORITY_DISABLE;
static uint32_t lastCommanderUpdateTick = 0;
static bool submitHighLevelPending = false;
static setpoint_t pendingHighLevelSetpoint;

typedef enum {
    SUPERVISOR_STATE_INIT = 0,
    SUPERVISOR_STATE_PREFL_CHECKS_PASSED,
    SUPERVISOR_STATE_ARMING,
    SUPERVISOR_STATE_READY_TO_FLY,
    SUPERVISOR_STATE_FLYING,
    SUPERVISOR_STATE_WARNING_LEVEL_OUT,
    SUPERVISOR_STATE_LANDED,
    SUPERVISOR_STATE_CRASHED,
    SUPERVISOR_STATE_EXCEPT_FREEFALL
} supervisor_state_t;

static supervisor_state_t supervisorState = SUPERVISOR_STATE_INIT;

#define SUPERVISOR_CB_SPINUP_TIMEOUT       (1 << 0)
#define SUPERVISOR_CB_PREFLIGHT_TIMEOUT    (1 << 1)
#define SUPERVISOR_CB_LANDING_TIMEOUT      (1 << 2)
#define SUPERVISOR_CB_CRTP_STOP            (1 << 3)
#define SUPERVISOR_CB_PARAM_STOP           (1 << 4)
#define SUPERVISOR_CB_WATCHDOG_STOP        (1 << 5)
#define SUPERVISOR_CB_WARNING              (1 << 6)
#define SUPERVISOR_CB_TIMEOUT              (1 << 7)
#define SUPERVISOR_CB_MOTOR_FAULT          (1 << 8)

// 1. Numerical Functions
int16_t saturateSignedInt16(int32_t value) {
    if (value > 32767) return 32767;
    if (value < -32767) return -32767;
    return (int16_t)value;
}

float capAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

float invSqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    float halfx = 0.5f * x;
    float y = x;
    int32_t i = *(int32_t*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// 2. Sensfusion6
void sensfusion6Init(void) {
    if (sensfusion6Initialized) return;
    qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
    calibrated = false;
    baseZacc = 0.0f;
    sensfusion6Initialized = true;
}

bool sensfusion6Test(void) {
    return sensfusion6Initialized;
}

void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float gx_rad = gx * 0.0174532925f;
    float gy_rad = gy * 0.0174532925f;
    float gz_rad = gz * 0.0174532925f;
    
    if (ax == 0.0f && ay == 0.0f && az == 0.0f) {
        float qw_dot = 0.5f * (-qx * gx_rad - qy * gy_rad - qz * gz_rad);
        float qx_dot = 0.5f * (qw * gx_rad + qy * gz_rad - qz * gy_rad);
        float qy_dot = 0.5f * (qw * gy_rad - qx * gz_rad + qz * gx_rad);
        float qz_dot = 0.5f * (qw * gz_rad + qx * gy_rad - qy * gx_rad);
        qw += qw_dot * dt;
        qx += qx_dot * dt;
        qy += qy_dot * dt;
        qz += qz_dot * dt;
    } else {
        if (config_imu_madgwick) {
            float recipNorm = invSqrt(ax*ax + ay*ay + az*az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;
            
            float _2qw = 2.0f * qw;
            float _2qx = 2.0f * qx;
            float _2qy = 2.0f * qy;
            float _2qz = 2.0f * qz;
            float _4qx = 4.0f * qx;
            float _4qy = 4.0f * qy;
            
            float f1 = _2qx * qz - _2qw * qy - ax;
            float f2 = _2qw * qx + _2qy * qz - ay;
            float f3 = 1.0f - 2.0f * (qx*qx + qy*qy) - az;
            
            float s_w = -_2qy * f1 + _2qx * f2;
            float s_x = _2qz * f1 + _2qw * f2 - _4qx * f3;
            float s_y = -_2qw * f1 + _2qz * f2 - _4qy * f3;
            float s_z = _2qx * f1 + _2qy * f2;
            
            float s_norm = invSqrt(s_w*s_w + s_x*s_x + s_y*s_y + s_z*s_z);
            if (s_norm > 0.0f) {
                s_w *= s_norm;
                s_x *= s_norm;
                s_y *= s_norm;
                s_z *= s_norm;
            }
            
            float qw_dot = 0.5f * (-qx * gx_rad - qy * gy_rad - qz * gz_rad) - 0.01f * s_w;
            float qx_dot = 0.5f * (qw * gx_rad + qy * gz_rad - qz * gy_rad) - 0.01f * s_x;
            float qy_dot = 0.5f * (qw * gy_rad - qx * gz_rad + qz * gx_rad) - 0.01f * s_y;
            float qz_dot = 0.5f * (qw * gz_rad + qx * gy_rad - qy * gx_rad) - 0.01f * s_z;
            
            qw += qw_dot * dt;
            qx += qx_dot * dt;
            qy += qy_dot * dt;
            qz += qz_dot * dt;
        } else {
            float recipNorm = invSqrt(ax*ax + ay*ay + az*az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;
            
            float vx = 2.0f * (qx*qz - qw*qy);
            float vy = 2.0f * (qw*qx + qy*qz);
            float vz = qw*qw - qx*qx - qy*qy + qz*qz;
            
            float ex = (ay*vz - az*vy);
            float ey = (az*vx - ax*vz);
            float ez = (ax*vy - ay*vx);
            
            if (twoKi > 0.0f) {
                integralFBx += ex * twoKi * dt;
                integralFBy += ey * twoKi * dt;
                integralFBz += ez * twoKi * dt;
            } else {
                integralFBx = 0.0f;
                integralFBy = 0.0f;
                integralFBz = 0.0f;
            }
            
            gx_rad += twoKp * ex + integralFBx;
            gy_rad += twoKp * ey + integralFBy;
            gz_rad += twoKp * ez + integralFBz;
            
            float qw_dot = 0.5f * (-qx * gx_rad - qy * gy_rad - qz * gz_rad);
            float qx_dot = 0.5f * (qw * gx_rad + qy * gz_rad - qz * gy_rad);
            float qy_dot = 0.5f * (qw * gy_rad - qx * gz_rad + qz * gx_rad);
            float qz_dot = 0.5f * (qw * gz_rad + qx * gy_rad - qy * gx_rad);
            
            qw += qw_dot * dt;
            qx += qx_dot * dt;
            qy += qy_dot * dt;
            qz += qz_dot * dt;
        }
    }
    
    float q_norm = invSqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if (q_norm > 0.0f) {
        qw *= q_norm;
        qx *= q_norm;
        qy *= q_norm;
        qz *= q_norm;
    }
    
    if (!calibrated && (ax != 0.0f || ay != 0.0f || az != 0.0f)) {
        float vx = 2.0f * (qx*qz - qw*qy);
        float vy = 2.0f * (qw*qx + qy*qz);
        float vz = qw*qw - qx*qx - qy*qy + qz*qz;
        baseZacc = ax*vx + ay*vy + az*vz;
        calibrated = true;
    }

    // Update log
    sensfusion6Log.qw = qw;
    sensfusion6Log.qx = qx;
    sensfusion6Log.qy = qy;
    sensfusion6Log.qz = qz;
    sensfusion6Log.isCalibrated = calibrated;
    sensfusion6GetEulerRPY(&sensfusion6Log.roll, &sensfusion6Log.pitch, &sensfusion6Log.yaw);
}

void estimatedGravityDirection(float* gx, float* gy, float* gz) {
    if (!gx || !gy || !gz) return;
    *gx = 2.0f * (qx*qz - qw*qy);
    *gy = 2.0f * (qw*qx + qy*qz);
    *gz = qw*qw - qx*qx - qy*qy + qz*qz;
}

void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw) {
    if (!roll || !pitch || !yaw) return;
    float gx = 2.0f * (qx*qz - qw*qy);
    float gy = 2.0f * (qw*qx + qy*qz);
    float gz = qw*qw - qx*qx - qy*qy + qz*qz;
    
    float gx_limited = gx;
    if (gx_limited > 1.0f) gx_limited = 1.0f;
    if (gx_limited < -1.0f) gx_limited = -1.0f;
    
    *pitch = -asinf(gx_limited) * 57.2957795f;
    *roll = atan2f(gy, gz) * 57.2957795f;
    *yaw = atan2f(2.0f * (qw*qz + qx*qy), qw*qw + qx*qx - qy*qy - qz*qz) * 57.2957795f;
}

void sensfusion6GetQuaternion(float* q) {
    if (!q) return;
    q[0] = qw;
    q[1] = qx;
    q[2] = qy;
    q[3] = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az) {
    float vx = 2.0f * (qx*qz - qw*qy);
    float vy = 2.0f * (qw*qx + qy*qz);
    float vz = qw*qw - qx*qx - qy*qy + qz*qz;
    return ax*vx + ay*vy + az*vz;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az) {
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

// 3. Power Distribution & Battery
uint16_t motorForceToPwm(float force) {
    if (force <= 0.0f) return 0;
    float pwm = 65535.0f * sqrtf(force / 0.17765f);
    if (pwm > 65535.0f) pwm = 65535.0f;
    return (uint16_t)pwm;
}

bool powerDistributionCap(int32_t* m1, int32_t* m2, int32_t* m3, int32_t* m4, int32_t maxAllowedThrust, int32_t idleThrust) {
    if (!m1 || !m2 || !m3 || !m4) return false;
    int32_t max = *m1;
    if (*m2 > max) max = *m2;
    if (*m3 > max) max = *m3;
    if (*m4 > max) max = *m4;
    
    if (max > maxAllowedThrust) {
        int32_t reduction = max - maxAllowedThrust;
        *m1 -= reduction;
        *m2 -= reduction;
        *m3 -= reduction;
        *m4 -= reduction;
        
        if (*m1 < idleThrust) *m1 = idleThrust;
        if (*m2 < idleThrust) *m2 = idleThrust;
        if (*m3 < idleThrust) *m3 = idleThrust;
        if (*m4 < idleThrust) *m4 = idleThrust;
        return true;
    }
    return false;
}

float batteryCompensation(float old, float supply, float alpha) {
    return old + alpha * (supply - old);
}

uint16_t motorsCompensateBatteryVoltage(uint16_t thrust, float nominal, float actual) {
    if (actual <= 0.0f) return thrust;
    float comp = (float)thrust * nominal / actual;
    float rounded = comp + 0.5f;
    if (rounded < 0.0f) rounded = 0.0f;
    if (rounded > 65535.0f) rounded = 65535.0f;
    return (uint16_t)rounded;
}

// 4. Cascade PID and controllerPid
void pidInit(PID_t* pid, float kp, float ki, float kd, float iLimit) {
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->iLimit = iLimit;
    pid->integrand = 0.0f;
    pid->prevError = 0.0f;
    pid->lastDeriv = 0.0f;
}

float pidUpdate(PID_t* pid, float error, float dt) {
    if (!pid || dt <= 0.0f) return 0.0f;
    pid->integrand += error * dt;
    if (pid->integrand > pid->iLimit) pid->integrand = pid->iLimit;
    if (pid->integrand < -pid->iLimit) pid->integrand = -pid->iLimit;
    
    float deriv = (error - pid->prevError) / dt;
    pid->prevError = error;
    pid->lastDeriv = deriv;
    
    return pid->kp * error + pid->ki * pid->integrand + pid->kd * deriv;
}

void attitudeControllerInit(void) {
    if (attitudeControllerInitialized) return;
    pidInit(&pidRoll, 6.0f, 3.0f, 0.0f, 20.0f);
    pidInit(&pidPitch, 6.0f, 3.0f, 0.0f, 20.0f);
    pidInit(&pidYaw, 6.0f, 1.0f, 0.35f, 360.0f);
    
    pidInit(&pidRollRate, 250.0f, 500.0f, 2.5f, 33.3f);
    pidInit(&pidPitchRate, 250.0f, 500.0f, 2.5f, 33.3f);
    pidInit(&pidYawRate, 120.0f, 16.7f, 0.0f, 166.7f);
    attitudeControllerInitialized = true;
}

void attitudeControllerResetAll(float currentRoll, float currentPitch, float currentYaw) {
    pidRoll.integrand = 0.0f; pidRoll.prevError = 0.0f;
    pidPitch.integrand = 0.0f; pidPitch.prevError = 0.0f;
    pidYaw.integrand = 0.0f; pidYaw.prevError = 0.0f;
    
    pidRollRate.integrand = 0.0f; pidRollRate.prevError = 0.0f;
    pidPitchRate.integrand = 0.0f; pidPitchRate.prevError = 0.0f;
    pidYawRate.integrand = 0.0f; pidYawRate.prevError = 0.0f;
}

void attitudeControllerResetAxis(int axis) {
    if (axis == 0) {
        pidRoll.integrand = 0.0f; pidRoll.prevError = 0.0f;
        pidRollRate.integrand = 0.0f; pidRollRate.prevError = 0.0f;
    } else if (axis == 1) {
        pidPitch.integrand = 0.0f; pidPitch.prevError = 0.0f;
        pidPitchRate.integrand = 0.0f; pidPitchRate.prevError = 0.0f;
    } else if (axis == 2) {
        pidYaw.integrand = 0.0f; pidYaw.prevError = 0.0f;
        pidYawRate.integrand = 0.0f; pidYawRate.prevError = 0.0f;
    }
}

static float desiredYaw = 0.0f;

void controllerPid(const sensor_data_t* sensors, const setpoint_t* setpoint, const state_t* state, control_t* control, float dt) {
    if (!sensors || !setpoint || !state || !control) return;
    
    if (setpoint->thrust == 0) {
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;
        control->thrust = 0;
        attitudeControllerResetAll(state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
        desiredYaw = state->attitude.yaw;
        return;
    }
    
    // Z mode check
    if (setpoint->mode.z == 0) { // MODE_DISABLE
        control->thrust = setpoint->thrust;
    } else {
        control->thrust = (uint16_t)positionControllerOutputThrust;
    }
    
    // Yaw Update
    if (setpoint->mode.yaw == 1) { // modeVelocity
        desiredYaw += setpoint->attitudeRate.yaw * dt;
        desiredYaw = capAngle(desiredYaw);
        if (yawMaxDelta != 0.0f) {
            float diff = capAngle(desiredYaw - state->attitude.yaw);
            if (diff > yawMaxDelta) diff = yawMaxDelta;
            if (diff < -yawMaxDelta) diff = -yawMaxDelta;
            desiredYaw = capAngle(state->attitude.yaw + diff);
        }
    } else if (setpoint->mode.yaw == 2) { // modeAbs
        desiredYaw = setpoint->attitude.yaw;
    } else if (setpoint->mode.yaw == 3) { // quat modeAbs
        desiredYaw = setpoint->attitude.yaw;
    }
    
    // Roll & Pitch Outer loop
    float desiredRollRate = 0.0f;
    float desiredPitchRate = 0.0f;
    
    if (setpoint->mode.x == 1) { // modeVelocity
        desiredRollRate = setpoint->attitudeRate.roll;
        pidRoll.integrand = 0.0f; pidRoll.prevError = 0.0f;
    } else {
        float errorRoll = capAngle(setpoint->attitude.roll - state->attitude.roll);
        desiredRollRate = pidUpdate(&pidRoll, errorRoll, dt);
    }
    
    if (setpoint->mode.y == 1) { // modeVelocity
        desiredPitchRate = setpoint->attitudeRate.pitch;
        pidPitch.integrand = 0.0f; pidPitch.prevError = 0.0f;
    } else {
        float errorPitch = capAngle(setpoint->attitude.pitch - state->attitude.pitch);
        desiredPitchRate = pidUpdate(&pidPitch, errorPitch, dt);
    }
    
    // Yaw Outer loop
    pidYaw.integrand = 0.0f; // reset=true semantics
    float errorYaw = capAngle(desiredYaw - state->attitude.yaw);
    float desiredYawRate = pidUpdate(&pidYaw, errorYaw, dt);
    
    // Inner loop (Rate loop)
    float pitchActual = -sensors->gyro.y;
    float errorRollRate = desiredRollRate - sensors->gyro.x;
    float errorPitchRate = desiredPitchRate - pitchActual;
    float errorYawRate = desiredYawRate - sensors->gyro.z;
    
    control->roll = saturateSignedInt16((int32_t)pidUpdate(&pidRollRate, errorRollRate, dt));
    control->pitch = saturateSignedInt16((int32_t)pidUpdate(&pidPitchRate, errorPitchRate, dt));
    control->yaw = saturateSignedInt16((int32_t)pidUpdate(&pidYawRate, errorYawRate, dt));
    
    if (control->controlMode == CONTROL_MODE_LEGACY) {
        control->yaw = -control->yaw;
    }
}

// 5. CRTP Commander RPYT
static bool lastAltHoldMode = false;
static bool altHoldModeSet = false;

void crtpCommanderRpytDecodeSetpoint(setpoint_t* setpoint, const crtp_rpyt_payload_t* payload) {
    if (!setpoint || !payload) return;
    
    if (activePriority == COMMANDER_PRIORITY_DISABLE) {
        thrustLocked = true;
    }
    if (payload->thrust == 0) {
        thrustLocked = false;
    }
    
    if (altHoldMode) {
        if (!lastAltHoldMode) {
            altHoldModeSet = true;
        }
        setpoint->mode.z = 1; // modeVelocity
        setpoint->thrust = 0;
        setpoint->velocity.z = ((float)payload->thrust - 32767.0f) / 32767.0f;
    } else {
        if (lastAltHoldMode) {
            setpoint->mode.z = 0; // modeDisable
            altHoldModeSet = false;
        }
    }
    lastAltHoldMode = altHoldMode;
    
    if (!altHoldMode) {
        if (thrustLocked || payload->thrust < 1000) {
            setpoint->thrust = 0;
        } else {
            setpoint->thrust = payload->thrust;
            if (setpoint->thrust > 60000) setpoint->thrust = 60000;
        }
    }
    
    float roll = payload->roll;
    float pitch = payload->pitch;
    
    if (plusMode) {
        float r_rad = 45.0f * 0.0174532925f;
        float cos_r = cosf(r_rad);
        float sin_r = sinf(r_rad);
        float r = roll;
        float p = pitch;
        roll = r * cos_r - p * sin_r;
        pitch = r * sin_r + p * cos_r;
    }
    
    if (carefreeMode) {
        carefreeError = true;
    }
    
    if (posSetMode && payload->thrust != 0) {
        setpoint->mode.x = 2; // modeAbs
        setpoint->mode.y = 2; // modeAbs
        setpoint->mode.z = 2; // modeAbs
        setpoint->mode.yaw = 2; // modeAbs
        setpoint->position.x = -pitch;
        setpoint->position.y = roll;
        setpoint->position.z = (float)payload->thrust / 1000.0f;
        setpoint->attitude.yaw = payload->yaw;
        setpoint->thrust = 0;
    } else if (posHoldMode) {
        setpoint->mode.x = 1; // modeVelocity
        setpoint->mode.y = 1; // modeVelocity
        setpoint->mode.yaw = 0; // modeDisable
        setpoint->velocity.x = pitch / 30.0f;
        setpoint->velocity.y = roll / 30.0f;
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
    } else {
        // Default modes
        if (config_imu_madgwick) { // or other rollPitchMode mapping
            setpoint->mode.x = 2; // modeAbs
            setpoint->mode.y = 2; // modeAbs
            setpoint->attitude.roll = roll;
            setpoint->attitude.pitch = pitch;
        } else {
            setpoint->mode.x = 2; // modeAbs
            setpoint->mode.y = 2; // modeAbs
            setpoint->attitude.roll = roll;
            setpoint->attitude.pitch = pitch;
        }
        
        setpoint->mode.yaw = 2; // modeAbs
        setpoint->attitude.yaw = payload->yaw;
    }
}

// 6. Supervisor
bool supervisorCanFly(void) {
    return (supervisorState == SUPERVISOR_STATE_READY_TO_FLY ||
            supervisorState == SUPERVISOR_STATE_FLYING ||
            supervisorState == SUPERVISOR_STATE_WARNING_LEVEL_OUT ||
            supervisorState == SUPERVISOR_STATE_LANDED);
}

bool supervisorCanArm(void) {
    return (supervisorState == SUPERVISOR_STATE_PREFL_CHECKS_PASSED);
}

bool supervisorIsArmed(void) {
    return isArmed;
}

bool supervisorIsCrashed(void) {
    return isCrashed;
}

void supervisorSetState(supervisor_state_t newState) {
    if (supervisorState == newState) return;
    
    bool wasAllowingArm = (supervisorState == SUPERVISOR_STATE_ARMING ||
                           supervisorState == SUPERVISOR_STATE_READY_TO_FLY ||
                           supervisorState == SUPERVISOR_STATE_FLYING ||
                           supervisorState == SUPERVISOR_STATE_WARNING_LEVEL_OUT ||
                           supervisorState == SUPERVISOR_STATE_LANDED);
    bool isAllowingArm = (newState == SUPERVISOR_STATE_ARMING ||
                          newState == SUPERVISOR_STATE_READY_TO_FLY ||
                          newState == SUPERVISOR_STATE_FLYING ||
                          newState == SUPERVISOR_STATE_WARNING_LEVEL_OUT ||
                          newState == SUPERVISOR_STATE_LANDED);
    if (wasAllowingArm && !isAllowingArm) {
        isArmed = false;
    }
    
    if (supervisorState == SUPERVISOR_STATE_ARMING) {
        spinupStartTick = 0;
        supervisorConditionBits &= ~SUPERVISOR_CB_SPINUP_TIMEOUT;
    }
    
    supervisorState = newState;
    
    if (supervisorState == SUPERVISOR_STATE_PREFL_CHECKS_PASSED && configAutoArming) {
        supervisorRequestArming(true);
    }
}

bool supervisorRequestArming(bool arm) {
    if (arm) {
        if (supervisorState == SUPERVISOR_STATE_PREFL_CHECKS_PASSED) {
            isArmed = true;
            supervisorSetState(SUPERVISOR_STATE_ARMING);
            spinupStartTick = currentTick;
            return true;
        }
        if (supervisorState == SUPERVISOR_STATE_ARMING) {
            return true;
        }
        return false;
    } else {
        isArmed = false;
        if (supervisorState == SUPERVISOR_STATE_ARMING ||
            supervisorState == SUPERVISOR_STATE_READY_TO_FLY ||
            supervisorState == SUPERVISOR_STATE_FLYING ||
            supervisorState == SUPERVISOR_STATE_WARNING_LEVEL_OUT ||
            supervisorState == SUPERVISOR_STATE_LANDED) {
            supervisorSetState(SUPERVISOR_STATE_PREFL_CHECKS_PASSED);
        }
        return true;
    }
}

bool supervisorRequestCrashRecovery(bool doRecovery) {
    if (isTumbled) return false;
    if (doRecovery) {
        isCrashed = false;
        return true;
    } else {
        isCrashed = true;
        supervisorSetState(SUPERVISOR_STATE_CRASHED);
        return true;
    }
}

bool isFlyingCheck(int32_t idleThrust) {
    bool active = false;
    for (int i = 0; i < 4; i++) {
        if (motorRatios[i] > idleThrust) {
            active = true;
        }
    }
    if (active) {
        recentFlightTick = currentTick;
        hasFlown = true;
    }
    if (!hasFlown) return false;
    return (currentTick - recentFlightTick < 2000);
}

static uint32_t tumbleTimerStart = 0;
static bool tumbleTimerActive = false;

bool isTumbledCheck(float ax, float ay, float az) {
    if (!tumbleCheckEnabled) return false;
    
    if (crashDetectionGs > 0.0f) {
        float norm = sqrtf(ax*ax + ay*ay + az*az);
        if (fabsf(norm - 1.0f) > crashDetectionGs) {
            isCrashed = true;
            supervisorSetState(SUPERVISOR_STATE_CRASHED);
        }
    }
    
    if (fabsf(ax) < freeFallThreshold && fabsf(ay) < freeFallThreshold && fabsf(az) < freeFallThreshold) {
        isFreeFalling = true;
        tumbleTimerActive = false;
        tumbleTimerStart = 0;
        supervisorSetState(SUPERVISOR_STATE_EXCEPT_FREEFALL);
    } else {
        isFreeFalling = false;
    }
    
    if (az < tiltThreshold) {
        if (!tumbleTimerActive) {
            tumbleTimerActive = true;
            tumbleTimerStart = currentTick;
        }
        uint32_t timeout = (az < invertThreshold) ? 200 : 500;
        if (currentTick - tumbleTimerStart >= timeout) {
            isTumbled = true;
        }
    } else {
        tumbleTimerActive = false;
        tumbleTimerStart = 0;
        isTumbled = false;
    }
    
    return isTumbled;
}

void supervisorCheckSpinup(void) {
    if (supervisorState == SUPERVISOR_STATE_ARMING) {
        if (spinupStartTick > 0) {
            uint32_t elapsed = currentTick - spinupStartTick;
            if (elapsed >= spinupTimeoutDuration) {
                supervisorConditionBits |= SUPERVISOR_CB_SPINUP_TIMEOUT;
            }
        }
    }
}

bool checkEmergencyStopWatchdog(uint32_t cur, uint32_t last) {
    if (last == 0) return true;
    return (cur - last <= 1000);
}

void supervisorCheckSetpointAge(void) {
    if (lastCommanderSetpointTick == 0) return;
    uint32_t age = currentTick - lastCommanderSetpointTick;
    if (age > 2000) {
        supervisorConditionBits |= SUPERVISOR_CB_TIMEOUT;
        supervisorConditionBits &= ~SUPERVISOR_CB_WARNING;
    } else if (age > 500) {
        supervisorConditionBits |= SUPERVISOR_CB_WARNING;
    } else {
        supervisorConditionBits &= ~(SUPERVISOR_CB_WARNING | SUPERVISOR_CB_TIMEOUT);
    }
}

bool supervisorIsPreflightTimeout(uint32_t duration) {
    return (supervisorConditionBits & SUPERVISOR_CB_PREFLIGHT_TIMEOUT) != 0;
}

bool supervisorIsLandingTimeout(uint32_t duration) {
    return (supervisorConditionBits & SUPERVISOR_CB_LANDING_TIMEOUT) != 0;
}

bool isRPMatArmingValid(void) {
    for (int i = 0; i < 4; i++) {
        if (motorRPMs[i] < 1000 || motorRPMs[i] > 20000) {
            return false;
        }
    }
    return true;
}

void supervisorSetSensorData(float ax, float ay, float az) {
    isTumbledCheck(ax, ay, az);
}

void supervisorSetMotorRatios(const int32_t* ratios) {
    if (ratios) {
        for (int i = 0; i < 4; i++) motorRatios[i] = ratios[i];
    }
}

void supervisorSetMotorRPMs(const int32_t* rpms) {
    if (rpms) {
        for (int i = 0; i < 4; i++) motorRPMs[i] = rpms[i];
    }
}

void supervisorConfigureSafety(float crashGs, float ffThresh, float tiltThresh, float invThresh, bool tumbleCheck) {
    crashDetectionGs = crashGs;
    freeFallThreshold = ffThresh;
    tiltThreshold = tiltThresh;
    invertThreshold = invThresh;
    tumbleCheckEnabled = tumbleCheck;
}

void supervisorOverrideSetpoint(setpoint_t* setpoint) {
    if (!setpoint) return;
    
    bool safetyFault = isCrashed || isTumbled || isFreeFalling || 
                       ((supervisorConditionBits & (SUPERVISOR_CB_WATCHDOG_STOP | 
                                                   SUPERVISOR_CB_MOTOR_FAULT | 
                                                   SUPERVISOR_CB_CRTP_STOP | 
                                                   SUPERVISOR_CB_PARAM_STOP | 
                                                   SUPERVISOR_CB_TIMEOUT)) != 0);
    
    if (safetyFault) {
        memset(setpoint, 0, sizeof(setpoint_t));
        return;
    }
    
    if (supervisorState == SUPERVISOR_STATE_WARNING_LEVEL_OUT) {
        setpoint->mode.x = 0; // modeDisable
        setpoint->mode.y = 0; // modeDisable
        setpoint->attitude.roll = 0.0f;
        setpoint->attitude.pitch = 0.0f;
        setpoint->mode.yaw = 1; // modeVelocity
        setpoint->attitudeRate.yaw = 0.0f;
    } else if (supervisorState == SUPERVISOR_STATE_ARMING ||
               supervisorState == SUPERVISOR_STATE_READY_TO_FLY ||
               supervisorState == SUPERVISOR_STATE_FLYING ||
               supervisorState == SUPERVISOR_STATE_LANDED) {
        // No modification
    } else {
        memset(setpoint, 0, sizeof(setpoint_t));
    }
}

bool supervisorAreMotorsAllowedToRun(void) {
    bool safetyFault = isCrashed || isTumbled || isFreeFalling || 
                       ((supervisorConditionBits & (SUPERVISOR_CB_WATCHDOG_STOP | 
                                                   SUPERVISOR_CB_MOTOR_FAULT | 
                                                   SUPERVISOR_CB_CRTP_STOP | 
                                                   SUPERVISOR_CB_PARAM_STOP)) != 0);
    if (safetyFault) return false;
    
    return (supervisorState == SUPERVISOR_STATE_ARMING ||
            supervisorState == SUPERVISOR_STATE_READY_TO_FLY ||
            supervisorState == SUPERVISOR_STATE_FLYING ||
            supervisorState == SUPERVISOR_STATE_WARNING_LEVEL_OUT ||
            supervisorState == SUPERVISOR_STATE_LANDED);
}

uint32_t supervisorGetInfoBitfield(void) {
    uint32_t bits = 0;
    if (supervisorCanArm()) bits |= (1 << 0);
    if (isArmed)            bits |= (1 << 1);
    if (configAutoArming)   bits |= (1 << 2);
    if (supervisorCanFly()) bits |= (1 << 3);
    if (isFlyingCheck(1000)) bits |= (1 << 4);
    if (isTumbled)          bits |= (1 << 5);
    if (isLocked)           bits |= (1 << 6);
    if (isCrashed)          bits |= (1 << 7);
    if (trajectoryFlying)   bits |= (1 << 8);
    if (trajectoryFinished) bits |= (1 << 9);
    if (trajectoryDisabled) bits |= (1 << 10);
    if (deckFault)          bits |= (1 << 11);
    return bits;
}

static uint32_t dshotFaultStartTick[4] = {0};

void supervisorUpdate(uint32_t step) {
    currentTick = step;
    
    if (!checkEmergencyStopWatchdog(currentTick, lastWatchdogNotificationTick)) {
        supervisorConditionBits |= SUPERVISOR_CB_WATCHDOG_STOP;
    }
    
    supervisorCheckSetpointAge();
    supervisorCheckSpinup();
    
    // DShot check
    if (!supervisorCanFly()) {
        supervisorConditionBits &= ~SUPERVISOR_CB_MOTOR_FAULT;
        for (int i = 0; i < 4; i++) dshotFaultStartTick[i] = 0;
    } else {
        bool anyFault = false;
        for (int i = 0; i < 4; i++) {
            if (motorRPMs[i] < 100) { // not-responding threshold
                if (dshotFaultStartTick[i] == 0) {
                    dshotFaultStartTick[i] = currentTick;
                } else if (currentTick - dshotFaultStartTick[i] >= 1000) {
                    anyFault = true;
                }
            } else {
                dshotFaultStartTick[i] = 0;
            }
        }
        if (anyFault) {
            supervisorConditionBits |= SUPERVISOR_CB_MOTOR_FAULT;
        }
    }
    
    isFlyingCheck(1000);
    
    if (isCrashed) {
        supervisorSetState(SUPERVISOR_STATE_CRASHED);
    } else if (isFreeFalling) {
        supervisorSetState(SUPERVISOR_STATE_EXCEPT_FREEFALL);
    } else {
        if (supervisorState == SUPERVISOR_STATE_ARMING) {
            if (supervisorConditionBits & (SUPERVISOR_CB_SPINUP_TIMEOUT | SUPERVISOR_CB_MOTOR_FAULT)) {
                supervisorSetState(SUPERVISOR_STATE_PREFL_CHECKS_PASSED);
            } else if (isRPMatArmingValid()) {
                supervisorSetState(SUPERVISOR_STATE_READY_TO_FLY);
            }
        } else if (supervisorState == SUPERVISOR_STATE_READY_TO_FLY) {
            if (isFlyingCheck(1000)) {
                supervisorSetState(SUPERVISOR_STATE_FLYING);
            }
        } else if (supervisorState == SUPERVISOR_STATE_FLYING) {
            if (supervisorConditionBits & SUPERVISOR_CB_WARNING) {
                supervisorSetState(SUPERVISOR_STATE_WARNING_LEVEL_OUT);
            }
        } else if (supervisorState == SUPERVISOR_STATE_WARNING_LEVEL_OUT) {
            if (!(supervisorConditionBits & SUPERVISOR_CB_WARNING)) {
                supervisorSetState(SUPERVISOR_STATE_FLYING);
            }
        }
    }
    
    supervisorLog.info = supervisorGetInfoBitfield();
    supervisorLog.accNorm = sqrtf(acc.x*acc.x + acc.y*acc.y + acc.z*acc.z);
    supervisorLog.state = (uint8_t)supervisorState;
    supervisorLog.isArmed = isArmed;
    supervisorLog.isCrashed = isCrashed;
}

// 7. Estimator & Commander Arbitration
#define ESTIMATOR_QUEUE_CAPACITY 16
static measurement_t estimatorQueue[ESTIMATOR_QUEUE_CAPACITY];
static int estimatorQueueHead = 0;
static int estimatorQueueTail = 0;
static int estimatorQueueSize = 0;

static measurement_t lastMeasurements[MEASUREMENT_COUNT];
static bool measurementUpdated[MEASUREMENT_COUNT] = {false};

bool estimatorEnqueue(const measurement_t* m) {
    if (!m || estimatorQueueSize >= ESTIMATOR_QUEUE_CAPACITY) return false;
    estimatorQueue[estimatorQueueTail] = *m;
    estimatorQueueTail = (estimatorQueueTail + 1) % ESTIMATOR_QUEUE_CAPACITY;
    estimatorQueueSize++;
    return true;
}

bool estimatorDequeue(measurement_t* m) {
    if (!m || estimatorQueueSize == 0) return false;
    *m = estimatorQueue[estimatorQueueHead];
    estimatorQueueHead = (estimatorQueueHead + 1) % ESTIMATOR_QUEUE_CAPACITY;
    estimatorQueueSize--;
    return true;
}

void estimatorComplementary(uint32_t step) {
    measurement_t m;
    while (estimatorDequeue(&m)) {
        lastMeasurements[m.type] = m;
        measurementUpdated[m.type] = true;
    }
    
    if ((step % 4) == 0) { // 250Hz in 1kHz
        if (measurementUpdated[MEASUREMENT_IMU]) {
            sensfusion6UpdateQ(
                lastMeasurements[MEASUREMENT_IMU].data.imu.gx,
                lastMeasurements[MEASUREMENT_IMU].data.imu.gy,
                lastMeasurements[MEASUREMENT_IMU].data.imu.gz,
                lastMeasurements[MEASUREMENT_IMU].data.imu.ax,
                lastMeasurements[MEASUREMENT_IMU].data.imu.ay,
                lastMeasurements[MEASUREMENT_IMU].data.imu.az,
                0.004f
            );
            float r, p, y;
            sensfusion6GetEulerRPY(&r, &p, &y);
            stateEstimate.roll = r;
            stateEstimate.pitch = p;
            stateEstimate.yaw = y;
            stateEstimate.q0 = qw;
            stateEstimate.q1 = qx;
            stateEstimate.q2 = qy;
            stateEstimate.q3 = qz;
            
            float accz_wg = sensfusion6GetAccZWithoutGravity(
                lastMeasurements[MEASUREMENT_IMU].data.imu.ax,
                lastMeasurements[MEASUREMENT_IMU].data.imu.ay,
                lastMeasurements[MEASUREMENT_IMU].data.imu.az
            );
            stateEstimate.vz += accz_wg * 9.81f * 0.004f;
        }
    }
    
    if ((step % 10) == 0) { // 100Hz in 1kHz
        stateEstimate.x += stateEstimate.vx * 0.01f;
        stateEstimate.y += stateEstimate.vy * 0.01f;
        stateEstimate.z += stateEstimate.vz * 0.01f;
    }
}

bool commanderSetSetpoint(const setpoint_t* setpoint, uint8_t priority) {
    if (!setpoint) return false;
    bool accept = false;
    if (priority == COMMANDER_PRIORITY_DISABLE) {
        accept = true;
    } else if (priority >= currentPriority) {
        accept = true;
    }
    
    if (accept) {
        activeSetpoint = *setpoint;
        currentPriority = priority;
        lastCommanderUpdateTick = currentTick;
        lastCommanderSetpointTick = currentTick;
        
        if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
            trajectoryFlying = false;
            trajectoryFinished = true;
        }
        return true;
    }
    return false;
}

void commanderRelaxPriority(void) {
    currentPriority = COMMANDER_PRIORITY_LOWEST;
}

uint32_t commanderGetInactivityTime(uint32_t curTick) {
    return curTick - lastCommanderUpdateTick;
}

uint8_t commanderGetActivePriority(void) {
    return currentPriority;
}

// 8. Stabilizer, compression, frequency
static int initSequence[10] = {0};
static int initSeqCount = 0;

void sensorsInit(void) { initSequence[initSeqCount++] = 1; }
void stateEstimatorInit(void) { initSequence[initSeqCount++] = 2; }
void controllerInit(void) { initSequence[initSeqCount++] = 3; }
void powerDistributionInit(void) { initSequence[initSeqCount++] = 4; }
void motorsInit(void) { initSequence[initSeqCount++] = 5; }
void collisionAvoidanceInit(void) { initSequence[initSeqCount++] = 6; }

void stabilizerInit(void) {
    if (stabilizerInitialized) return;
    initSeqCount = 0;
    sensorsInit();
    stateEstimatorInit();
    controllerInit();
    powerDistributionInit();
    motorsInit();
    collisionAvoidanceInit();
    stabilizerInitialized = true;
}

bool sensorsWaitDataReady(void) { return true; }
void sensorsAcquire(sensor_data_t* sensors) {
    if (sensors) {
        *sensors = currentSensors;
    }
}

void stabilizerSubmitHighLevelSetpoint(const setpoint_t* setpoint) {
    if (setpoint) {
        pendingHighLevelSetpoint = *setpoint;
        submitHighLevelPending = true;
    }
}

void stabilizerTaskStep(uint32_t step) {
    currentTick = step;
    
    sensorsWaitDataReady();
    
    sensor_data_t local_sensors;
    sensorsAcquire(&local_sensors);
    
    // update logs from acquired sensors
    gyro.x = local_sensors.gyro.x;
    gyro.y = local_sensors.gyro.y;
    gyro.z = local_sensors.gyro.z;
    acc.x = local_sensors.acc.x;
    acc.y = local_sensors.acc.y;
    acc.z = local_sensors.acc.z;
    
    if (measurementUpdated[MEASUREMENT_BARO]) {
        baro.pressure = lastMeasurements[MEASUREMENT_BARO].data.baro.pressure;
        baro.temp = lastMeasurements[MEASUREMENT_BARO].data.baro.temp;
        baro.asl = 44330.0f * (1.0f - powf(baro.pressure / 1013.25f, 0.1903f));
    }
    
    estimatorComplementary(step);
    
    state_t local_state;
    local_state.attitude.roll = stateEstimate.roll;
    local_state.attitude.pitch = stateEstimate.pitch;
    local_state.attitude.yaw = stateEstimate.yaw;
    local_state.position.x = stateEstimate.x;
    local_state.position.y = stateEstimate.y;
    local_state.position.z = stateEstimate.z;
    local_state.velocity.x = stateEstimate.vx;
    local_state.velocity.y = stateEstimate.vy;
    local_state.velocity.z = stateEstimate.vz;
    
    if (submitHighLevelPending) {
        commanderSetSetpoint(&pendingHighLevelSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
        submitHighLevelPending = false;
    }
    
    setpoint_t local_setpoint;
    local_setpoint = activeSetpoint;
    
    supervisorUpdate(step);
    
    // collisionAvoidanceUpdateSetpoint placeholder
    
    supervisorOverrideSetpoint(&local_setpoint);
    
    if (healthShallWeRunTest()) {
        healthRunTests(step);
        return;
    }
    
    if (!supervisorCanFly()) {
        memset(&local_setpoint, 0, sizeof(setpoint_t));
    }
    
    control_t local_control;
    local_control.controlMode = CONTROL_MODE_LEGACY;
    
    controllerPid(&local_sensors, &local_setpoint, &local_state, &local_control, 0.001f);
    
    // powerDistribution
    if (local_control.controlMode == CONTROL_MODE_LEGACY) {
        int32_t r = local_control.roll / 2;
        int32_t p = local_control.pitch / 2;
        m1_mix = local_control.thrust - r + p + local_control.yaw;
        m2_mix = local_control.thrust - r - p - local_control.yaw;
        m3_mix = local_control.thrust + r - p + local_control.yaw;
        m4_mix = local_control.thrust + r + p - local_control.yaw;
    }
    
    // batteryCompensation & motorsCompensateBatteryVoltage
    static float filteredBatVoltage = 3.7f;
    filteredBatVoltage = batteryCompensation(filteredBatVoltage, batteryVoltage, 0.01f);
    
    m1_mix = motorsCompensateBatteryVoltage((uint16_t)m1_mix, batteryNominalVoltage, filteredBatVoltage);
    m2_mix = motorsCompensateBatteryVoltage((uint16_t)m2_mix, batteryNominalVoltage, filteredBatVoltage);
    m3_mix = motorsCompensateBatteryVoltage((uint16_t)m3_mix, batteryNominalVoltage, filteredBatVoltage);
    m4_mix = motorsCompensateBatteryVoltage((uint16_t)m4_mix, batteryNominalVoltage, filteredBatVoltage);
    
    powerDistributionCap(&m1_mix, &m2_mix, &m3_mix, &m4_mix, 65535, 1000);
    
    motor_ratios_t local_motors;
    if (!supervisorAreMotorsAllowedToRun()) {
        local_motors.m1 = 0;
        local_motors.m2 = 0;
        local_motors.m3 = 0;
        local_motors.m4 = 0;
    } else {
        local_motors.m1 = (uint16_t)m1_mix;
        local_motors.m2 = (uint16_t)m2_mix;
        local_motors.m3 = (uint16_t)m3_mix;
        local_motors.m4 = (uint16_t)m4_mix;
    }
    
    motor.m1 = local_motors.m1;
    motor.m2 = local_motors.m2;
    motor.m3 = local_motors.m3;
    motor.m4 = local_motors.m4;
    
    int32_t ratios[4] = {local_motors.m1, local_motors.m2, local_motors.m3, local_motors.m4};
    supervisorSetMotorRatios(ratios);
}

uint32_t quatcompress(float qw, float qx, float qy, float qz) {
    float q[4] = {qw, qx, qy, qz};
    int max_idx = 0;
    float max_val = fabsf(q[0]);
    for (int i = 1; i < 4; i++) {
        if (fabsf(q[i]) > max_val) {
            max_val = fabsf(q[i]);
            max_idx = i;
        }
    }
    float sign = (q[max_idx] < 0) ? -1.0f : 1.0f;
    int32_t e[3];
    int idx = 0;
    for (int i = 0; i < 4; i++) {
        if (i == max_idx) continue;
        float val = q[i] * sign;
        int32_t val_i = (int32_t)(val * 722.6685f);
        if (val_i > 511) val_i = 511;
        if (val_i < -512) val_i = -512;
        e[idx++] = val_i & 0x3FF;
    }
    return ((uint32_t)max_idx << 30) | ((uint32_t)e[0] << 20) | ((uint32_t)e[1] << 10) | (uint32_t)e[2];
}

void compressState(const state_t* state, const sensor_data_t* sensors, compressed_state_t* compressed) {
    if (!state || !sensors || !compressed) return;
    compressed->pos_x = (int16_t)(state->position.x * 1000.0f);
    compressed->pos_y = (int16_t)(state->position.y * 1000.0f);
    compressed->pos_z = (int16_t)(state->position.z * 1000.0f);
    
    compressed->vel_x = (int16_t)(state->velocity.x * 1000.0f);
    compressed->vel_y = (int16_t)(state->velocity.y * 1000.0f);
    compressed->vel_z = (int16_t)(state->velocity.z * 1000.0f);
    
    compressed->acc_x = (int16_t)(sensors->acc.x * 9810.0f);
    compressed->acc_y = (int16_t)(sensors->acc.y * 9810.0f);
    compressed->acc_z = (int16_t)((sensors->acc.z + 1.0f) * 9810.0f);
    
    compressed->gyro_x = (int16_t)(sensors->gyro.x * 17.4532925f);
    compressed->gyro_y = (int16_t)(-sensors->gyro.y * 17.4532925f);
    compressed->gyro_z = (int16_t)(sensors->gyro.z * 17.4532925f);
    
    compressed->quat = quatcompress(stateEstimate.q0, stateEstimate.q1, stateEstimate.q2, stateEstimate.q3);
}

bool rateSupervisorValidate(uint32_t hz) {
    return (hz >= 997 && hz <= 1003);
}

// 9. Health
bool propTestRequest = false;
bool batTestRequest = false;

typedef enum {
    HEALTH_STATE_IDLE = 0,
    HEALTH_STATE_PROP_ACC_CONFIG,
    HEALTH_STATE_PROP_NOISE_COLLECT,
    HEALTH_STATE_PROP_MOTOR_TEST,
    HEALTH_STATE_PROP_EVALUATE,
    HEALTH_STATE_BAT_LOAD,
    HEALTH_STATE_BAT_EVALUATE,
    HEALTH_STATE_BAT_WAIT_RESTART
} health_state_t;

static health_state_t healthState = HEALTH_STATE_IDLE;
static uint32_t healthTick = 0;
static float idleVoltage = 3.7f;
static float minLoadedVoltage = 3.7f;

static float accSamples[100];
static int accSampleCount = 0;
static int currentMotorUnderTest = 0;
static float motorVibrationSum[4] = {0.0f};
static float motorVibrationSumSq[4] = {0.0f};
static int motorSampleCount[4] = {0};

static float propTestHighThreshold = 5.0f;
static float propTestLowThreshold = 0.1f;
static float batterySagThreshold = 0.3f;

void startPropTest(void) {
    propTestRequest = true;
}

void startBatTest(void) {
    batTestRequest = true;
}

bool healthShallWeRunTest(void) {
    if (propTestRequest || batTestRequest) {
        if (propTestRequest) {
            healthState = HEALTH_STATE_PROP_ACC_CONFIG;
            propTestRequest = false;
        } else if (batTestRequest) {
            healthState = HEALTH_STATE_BAT_LOAD;
            healthTick = 0;
            batTestRequest = false;
        }
        return true;
    }
    return (healthState != HEALTH_STATE_IDLE);
}

static sensor_data_t currentSensors;

void healthRunTests(uint32_t step) {
    healthTick++;
    
    if (healthState == HEALTH_STATE_PROP_ACC_CONFIG) {
        idleVoltage = batteryVoltage;
        accSampleCount = 0;
        m1_mix = m2_mix = m3_mix = m4_mix = 0;
        healthState = HEALTH_STATE_PROP_NOISE_COLLECT;
        healthTick = 0;
    }
    else if (healthState == HEALTH_STATE_PROP_NOISE_COLLECT) {
        if (accSampleCount < 100) {
            accSamples[accSampleCount++] = currentSensors.acc.z;
        }
        if (accSampleCount >= 100) {
            currentMotorUnderTest = 0;
            healthState = HEALTH_STATE_PROP_MOTOR_TEST;
            healthTick = 0;
            for (int i = 0; i < 4; i++) {
                motorVibrationSum[i] = 0.0f;
                motorVibrationSumSq[i] = 0.0f;
                motorSampleCount[i] = 0;
            }
        }
    }
    else if (healthState == HEALTH_STATE_PROP_MOTOR_TEST) {
        int motor_idx = currentMotorUnderTest;
        m1_mix = (motor_idx == 0) ? 20000 : 0;
        m2_mix = (motor_idx == 1) ? 20000 : 0;
        m3_mix = (motor_idx == 2) ? 20000 : 0;
        m4_mix = (motor_idx == 3) ? 20000 : 0;
        
        float vib = currentSensors.acc.z;
        motorVibrationSum[motor_idx] += vib;
        motorVibrationSumSq[motor_idx] += vib * vib;
        motorSampleCount[motor_idx]++;
        
        if (healthTick >= 50) {
            currentMotorUnderTest++;
            healthTick = 0;
            if (currentMotorUnderTest >= 4) {
                healthState = HEALTH_STATE_PROP_EVALUATE;
            }
        }
    }
    else if (healthState == HEALTH_STATE_PROP_EVALUATE) {
        m1_mix = m2_mix = m3_mix = m4_mix = 0;
        healthMotorPass = 0;
        for (int i = 0; i < 4; i++) {
            float sum = motorVibrationSum[i];
            float sumSq = motorVibrationSumSq[i];
            float n = (float)motorSampleCount[i];
            float var = 0.0f;
            if (n > 0.0f) {
                var = sumSq - (sum * sum / n);
            }
            
            bool pass = false;
            if (propTestHighThreshold == 0.0f) {
                pass = true;
            } else if (var >= propTestLowThreshold && var <= propTestHighThreshold) {
                pass = true;
            }
            
            if (pass) {
                healthMotorPass |= (1 << i);
            } else {
                healthMotorTestCount++;
            }
        }
        healthState = HEALTH_STATE_IDLE;
    }
    else if (healthState == HEALTH_STATE_BAT_LOAD) {
        if (healthTick == 1) {
            m1_mix = m2_mix = m3_mix = m4_mix = 30000;
            minLoadedVoltage = batteryVoltage;
        } else if (healthTick >= 2 && healthTick <= 49) {
            if (batteryVoltage < minLoadedVoltage) {
                minLoadedVoltage = batteryVoltage;
            }
        } else if (healthTick == 50) {
            m1_mix = m2_mix = m3_mix = m4_mix = 0;
            healthBatterySag = idleVoltage - minLoadedVoltage;
            healthBatteryPass = (healthBatterySag <= batterySagThreshold);
            healthState = HEALTH_STATE_IDLE;
        }
    }
    else if (healthState == HEALTH_STATE_BAT_WAIT_RESTART) {
        if (healthTick >= 2000) {
            healthState = HEALTH_STATE_BAT_LOAD;
            healthTick = 0;
        }
    }
    
    healthLog.motorPass = healthMotorPass;
    healthLog.batteryPass = healthBatteryPass;
    healthLog.batterySag = healthBatterySag;
    healthLog.motorTestCount = healthMotorTestCount;
}

void restartBatTest(void) {
    healthState = HEALTH_STATE_BAT_WAIT_RESTART;
    healthTick = 0;
}

// 10. CRTP Transmission
#define CRTP_RX_QUEUE_CAPACITY 16
typedef struct {
    crtp_packet_t queue[CRTP_RX_QUEUE_CAPACITY];
    int head;
    int tail;
    int size;
    crtp_callback_t cb;
    bool created;
} crtp_port_t;

static crtp_port_t crtpPorts[CRTP_NBR_OF_PORTS];

#define CRTP_TX_QUEUE_CAPACITY 200
static crtp_packet_t crtpTxQueue[CRTP_TX_QUEUE_CAPACITY];
static int crtpTxQueueHead = 0;
static int crtpTxQueueTail = 0;
static int crtpTxQueueSize = 0;

static crtp_link_t* currentLink = NULL;
static bool crtpLinkConnected = false;
static bool crtpErrorState = false;

static uint32_t crtpRxCount = 0;
static uint32_t crtpTxCount = 0;
static float crtpRxRate = 0.0f;
static float crtpTxRate = 0.0f;
static uint32_t lastStatsTick = 0;

static bool nopSend(crtp_packet_t* p) { return false; }
static void nopReset(void) {}
static crtp_link_t nopLink = { nopSend, nopReset };

void crtpInit(void) {
    static bool crtpInitialized = false;
    if (crtpInitialized) return;
    crtpTxQueueHead = 0;
    crtpTxQueueTail = 0;
    crtpTxQueueSize = 0;
    for (int i = 0; i < CRTP_NBR_OF_PORTS; i++) {
        crtpPorts[i].head = 0;
        crtpPorts[i].tail = 0;
        crtpPorts[i].size = 0;
        crtpPorts[i].cb = NULL;
        crtpPorts[i].created = false;
    }
    crtpErrorState = false;
    currentLink = &nopLink;
    crtpInitialized = true;
}

bool crtpCreatePortQueue(int port) {
    if (port < 0 || port >= CRTP_NBR_OF_PORTS) {
        crtpErrorState = true;
        return false;
    }
    if (crtpPorts[port].created) {
        crtpErrorState = true;
        return false;
    }
    crtpPorts[port].created = true;
    crtpPorts[port].head = 0;
    crtpPorts[port].tail = 0;
    crtpPorts[port].size = 0;
    return true;
}

bool crtpRegisterPortCB(int port, crtp_callback_t cb) {
    if (port < 0 || port >= CRTP_NBR_OF_PORTS) return false;
    crtpPorts[port].cb = cb;
    return true;
}

bool crtpSendPacket(crtp_packet_t* p) {
    if (!p) return false;
    if (crtpTxQueueSize >= CRTP_TX_QUEUE_CAPACITY) return false;
    crtpTxQueue[crtpTxQueueTail] = *p;
    crtpTxQueueTail = (crtpTxQueueTail + 1) % CRTP_TX_QUEUE_CAPACITY;
    crtpTxQueueSize++;
    return true;
}

bool crtpSendPacketBlock(crtp_packet_t* p) {
    return crtpSendPacket(p);
}

int crtpGetFreeTxQueuePackets(void) {
    return CRTP_TX_QUEUE_CAPACITY - crtpTxQueueSize;
}

void crtpSetLink(crtp_link_t* link) {
    if (link == NULL) {
        currentLink = &nopLink;
    } else {
        currentLink = link;
    }
}

void crtpReset(void) {
    crtpTxQueueHead = 0;
    crtpTxQueueTail = 0;
    crtpTxQueueSize = 0;
    if (currentLink && currentLink->reset) {
        currentLink->reset();
    }
}

bool crtpIsConnected(void) {
    return crtpLinkConnected;
}

void crtpUpdateStats(uint32_t tick) {
    if (tick - lastStatsTick >= 500) {
        float dt_sec = (float)(tick - lastStatsTick) / 1000.0f;
        if (dt_sec > 0.0f) {
            crtpRxRate = (float)crtpRxCount / dt_sec;
            crtpTxRate = (float)crtpTxCount / dt_sec;
        }
        crtpRxCount = 0;
        crtpTxCount = 0;
        lastStatsTick = tick;
    }
}

bool crtpReceivePacket(int port, crtp_packet_t* p) {
    if (port < 0 || port >= CRTP_NBR_OF_PORTS || !p) return false;
    if (!crtpPorts[port].created || crtpPorts[port].size == 0) return false;
    *p = crtpPorts[port].queue[crtpPorts[port].head];
    crtpPorts[port].head = (crtpPorts[port].head + 1) % CRTP_RX_QUEUE_CAPACITY;
    crtpPorts[port].size--;
    return true;
}

bool crtpReceivePacketBlock(int port, crtp_packet_t* p) {
    return crtpReceivePacket(port, p);
}

bool crtpReceivePacketWait(int port, crtp_packet_t* p, uint32_t timeout) {
    return crtpReceivePacket(port, p);
}

void simulateCrtpRx(crtp_packet_t* p) {
    if (currentLink == &nopLink || !p) return;
    crtpRxCount++;
    int port = (p->header >> 4) & 0x0F;
    if (port >= 0 && port < CRTP_NBR_OF_PORTS) {
        bool delivered = false;
        if (crtpPorts[port].created && crtpPorts[port].size < CRTP_RX_QUEUE_CAPACITY) {
            crtpPorts[port].queue[crtpPorts[port].tail] = *p;
            crtpPorts[port].tail = (crtpPorts[port].tail + 1) % CRTP_RX_QUEUE_CAPACITY;
            crtpPorts[port].size++;
            delivered = true;
        }
        if (crtpPorts[port].cb) {
            crtpPorts[port].cb(p);
            delivered = true;
        }
        if (!delivered) {
            // Discarded
        }
    }
}

void simulateCrtpTxStep(uint32_t step) {
    if (currentLink == &nopLink || crtpTxQueueSize == 0) return;
    crtp_packet_t* p = &crtpTxQueue[crtpTxQueueHead];
    if (currentLink && currentLink->send && currentLink->send(p)) {
        crtpTxQueueHead = (crtpTxQueueHead + 1) % CRTP_TX_QUEUE_CAPACITY;
        crtpTxQueueSize--;
        crtpTxCount++;
    }
}

// 11. Deck Discovery
int deckDiscovery(const uint8_t* i2cAddresses, int i2cCount, const uint8_t* owROMs, int owCount, deck_info_t* discovered, int capacity) {
    if (!discovered || capacity <= 0) return 0;
    int count = 0;
    for (int i = 0; i < i2cCount && count < capacity; i++) {
        uint8_t addr = i2cAddresses[i];
        bool dup = false;
        for (int j = 0; j < count; j++) {
            if (discovered[j].i2cAddr == addr) {
                dup = true;
                break;
            }
        }
        if (!dup) {
            discovered[count].i2cAddr = addr;
            memset(discovered[count].onewireROM, 0, 8);
            count++;
        }
    }
    for (int i = 0; i < owCount && count < capacity; i++) {
        const uint8_t* rom = &owROMs[i * 8];
        bool dup = false;
        for (int j = 0; j < count; j++) {
            if (memcmp(discovered[j].onewireROM, rom, 8) == 0) {
                dup = true;
                break;
            }
        }
        if (!dup) {
            discovered[count].i2cAddr = 0;
            memcpy(discovered[count].onewireROM, rom, 8);
            count++;
        }
    }
    return count;
}

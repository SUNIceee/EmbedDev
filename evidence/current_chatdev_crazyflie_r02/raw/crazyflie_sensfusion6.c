/* DOCSTRING: Numerical utilities and Sensfusion6 quaternion/Euler implementation. */
#include "6_generated_code.h"
#include "crazyflie_internal.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
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

int16_t saturateSignedInt16(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }
    if (value < -32767) {
        return -32767;
    }
    return (int16_t)value;
}

float capAngle(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

float invSqrt(float x)
{
    if (x <= 0.0f) {
        return 0.0f;
    }

    float xhalf = 0.5f * x;
    uint32_t i = 0u;
    float y = x;

    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759dfu - (i >> 1u);
    memcpy(&y, &i, sizeof(y));

    y = y * (1.5f - xhalf * y * y);
    return y;
}

void estimatedGravityDirection(float qw_, float qx_, float qy_, float qz_,
                               float *gravX, float *gravY, float *gravZ)
{
    if (gravX == NULL || gravY == NULL || gravZ == NULL) {
        return;
    }

    *gravX = 2.0f * (qx_ * qz_ - qw_ * qy_);
    *gravY = 2.0f * (qw_ * qx_ + qy_ * qz_);
    *gravZ = qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_;

    gravityX = *gravX;
    gravityY = *gravY;
    gravityZ = *gravZ;
}

static void sensfusion6UpdateLog(void)
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

void sensfusion6Init(void)
{
    if (sensfusion6IsInit) {
        return;
    }

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

    twoKp = 0.8f;
    twoKi = 0.002f;
    beta = 0.01f;
    baseZacc = 0.0f;

    sensfusion6IsCalibrated = false;
    sensfusion6IsInit = true;

    sensfusion6UpdateLog();
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

    if (dt <= 0.0f) {
        return;
    }

    float recipNorm = 0.0f;
    float halfvx = 0.0f;
    float halfvy = 0.0f;
    float halfvz = 0.0f;
    float halfex = 0.0f;
    float halfey = 0.0f;
    float halfez = 0.0f;

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) {
        /* Zero acceleration: skip acc correction and prevent division by zero. */
    } else {
        recipNorm = 1.0f / norm;
        float axn = ax * recipNorm;
        float ayn = ay * recipNorm;
        float azn = az * recipNorm;

        halfvx = qx * qz - qw * qy;
        halfvy = qw * qx + qy * qz;
        halfvz = qw * qw - qx * qx - qy * qy + qz * qz;

        halfex = (ayn * halfvz - azn * halfvy);
        halfey = (azn * halfvx - axn * halfvz);
        halfez = (axn * halfvy - ayn * halfvx);

        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += twoKp * halfex + integralFBx;
        gy += twoKp * halfey + integralFBy;
        gz += twoKp * halfez + integralFBz;
    }

    float oldQw = qw;
    float oldQx = qx;
    float oldQy = qy;
    float oldQz = qz;

    qw += 0.5f * dt * (-qx * gx - qy * gy - qz * gz);
    qx += 0.5f * dt * (oldQw * gx + oldQy * gz - oldQz * gy);
    qy += 0.5f * dt * (oldQw * gy - oldQx * gz + oldQz * gx);
    qz += 0.5f * dt * (oldQw * gz + oldQx * gy - oldQy * gx);

    norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm < 1e-6f) {
        qw = 1.0f;
        qx = 0.0f;
        qy = 0.0f;
        qz = 0.0f;
    } else {
        recipNorm = 1.0f / norm;
        qw *= recipNorm;
        qx *= recipNorm;
        qy *= recipNorm;
        qz *= recipNorm;
    }

    estimatedGravityDirection(qw, qx, qy, qz, &gravityX, &gravityY, &gravityZ);

    if (!sensfusion6IsCalibrated && norm >= 1e-6f) {
        baseZacc = ax * gravityX + ay * gravityY + az * gravityZ;
        sensfusion6IsCalibrated = true;
    }

    sensfusion6UpdateLog();
}

void sensfusion6GetEulerRPY(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (roll_deg == NULL || pitch_deg == NULL || yaw_deg == NULL) {
        return;
    }

    float gx = 2.0f * (qx * qz - qw * qy);
    float gy = 2.0f * (qw * qx + qy * qz);
    float gz = qw * qw - qx * qx - qy * qy + qz * qz;

    float clampPitch = (gx < -1.0f) ? -1.0f : ((gx > 1.0f) ? 1.0f : gx);
    float roll_rad = atan2f(gy, gz);
    float pitch_rad = asinf(clampPitch);
    float yaw_rad = atan2f(2.0f * (qw * qz + qx * qy),
                           1.0f - 2.0f * (qy * qy + qz * qz));

    *roll_deg = roll_rad * (180.0f / (float)M_PI);
    *pitch_deg = pitch_rad * (180.0f / (float)M_PI);
    *yaw_deg = yaw_rad * (180.0f / (float)M_PI);
}

void sensfusion6GetQuaternion(float *qw_out, float *qx_out, float *qy_out, float *qz_out)
{
    if (qw_out == NULL || qx_out == NULL || qy_out == NULL || qz_out == NULL) {
        return;
    }

    *qw_out = qw;
    *qx_out = qx;
    *qy_out = qy;
    *qz_out = qz;
}

float sensfusion6GetAccZ(float ax, float ay, float az)
{
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 1.0f;
    estimatedGravityDirection(qw, qx, qy, qz, &gx, &gy, &gz);
    return ax * gx + ay * gy + az * gz;
}

float sensfusion6GetAccZWithoutGravity(float ax, float ay, float az)
{
    return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

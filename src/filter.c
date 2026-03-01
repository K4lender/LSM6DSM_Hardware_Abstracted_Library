/*
 * Title: filter.c
 *
 * Date: 17/02/2026
 *
 * Author: Muhammed Selman Çetin
 *
 * GitHub: K4lender
 */


#include "filter.h"


///////////////////////////////// Low Pass Filter //////////////////////////////////////////

void LPF_Init(LowPassFilter_t* lpf, float cutoffFreq, float sampleTime) {
    float wc;
    
    if (lpf == NULL) {
        return;
    }
    
    wc = 2.0f * M_PI * cutoffFreq;
    
    lpf->coeffNum = 1.6221f * wc * wc * sampleTime * sampleTime;
    lpf->coeffDen[0] = 1.0f / (1.0f + 2.206f * wc * sampleTime + lpf->coeffNum);
    lpf->coeffDen[1] = -(2.0f + 2.206f * wc * sampleTime);
    
    lpf->buf[0] = 0.0f;
    lpf->buf[1] = 0.0f;
    lpf->out = 0.0f;
}


float LPF_Update(LowPassFilter_t* lpf, float input) {
    if (lpf == NULL) {
        return 0.0f;
    }
    
    lpf->buf[1] = lpf->buf[0];
    lpf->buf[0] = lpf->out;
    
    lpf->out = lpf->coeffDen[0] * (lpf->coeffNum * input - (lpf->coeffDen[1] * lpf->buf[0] + lpf->buf[1]));
    
    return lpf->out;
}


void LPF_Reset(LowPassFilter_t* lpf) {
    if (lpf == NULL) {
        return;
    }
    
    lpf->buf[0] = 0.0f;
    lpf->buf[1] = 0.0f;
    lpf->out = 0.0f;
}


/////////////////////////////////// Notch Filter //////////////////////////////////////////

void Notch_Init(NotchFilter_t* nf, float centerFreq, float notchWidth, float sampleTime) {

    if (nf == NULL) {
        return;
    }
    
    float w0 = 2.0f * M_PI * centerFreq;
    float ww = 2.0f * M_PI * notchWidth;
    
    float w0_pw = (2.0f / sampleTime) * tanf(0.5f * w0 * sampleTime);
    
    nf->alpha = 4.0f + ((w0_pw * w0_pw) * (sampleTime * sampleTime));
    nf->beta = 2.0f + (ww * sampleTime);
    
    for (int i = 0; i < 3; i++) {
        nf->x[i] = 0.0f;
        nf->y[i] = 0.0f;
    }
}


float Notch_Update(NotchFilter_t* nf, float input) {
    if (nf == NULL) {
        return 0.0f;
    }
    
    nf->x[2] = nf->x[1];
    nf->x[1] = nf->x[0];
    nf->y[2] = nf->y[1];
    nf->y[1] = nf->y[0];
    
    nf->x[0] = input;
    
    nf->y[0] = (nf->alpha * nf->x[0] + 2.0f * (nf->alpha - 8.0f) * nf->x[1] 
               + nf->alpha * nf->x[2] - 2.0f * (nf->alpha - 8.0f) * nf->y[1] 
               - (nf->alpha - nf->beta) * nf->y[2]) / (nf->alpha + nf->beta);
    
    return nf->y[0];
}


void Notch_Reset(NotchFilter_t* nf) {

    if (nf == NULL) {
        return;
    }
    
    for (int i = 0; i < 3; i++) {
        nf->x[i] = 0.0f;
        nf->y[i] = 0.0f;
    }
}


////////////////////////////////// Complementary Filter //////////////////////////////////////////

void CF_Init(ComplementaryFilter_t* cf, float alpha, float sampleTime) {
    if (cf == NULL) {
        return;
    }
    
    cf->alpha = alpha;
    cf->dt = sampleTime;
    cf->angle = 0.0f;
}


float CF_Update(ComplementaryFilter_t* cf, float gyro_rate, float accel_angle) {
    if (cf == NULL) {
        return 0.0f;
    }
    
    cf->angle = cf->alpha * (cf->angle + gyro_rate * cf->dt) + (1.0f - cf->alpha) * accel_angle;
    
    return cf->angle;
}


void CF_Reset(ComplementaryFilter_t* cf) {
    if (cf == NULL) {
        return;
    }
    
    cf->angle = 0.0f;
}


////////////////////////////////// Mahony Filter //////////////////////////////////////////

#define DEG_TO_RAD_F  (0.017453292519943f)   // pi / 180
#define RAD_TO_DEG_F  (57.295779513082f)      // 180 / pi

void Mahony_Init(MahonyFilter_t* mf, float kp, float ki, float sampleTime) {
    if (mf == NULL) {
    	return;
    }

    mf->q0 = 1.0f; mf->q1 = 0.0f; mf->q2 = 0.0f; mf->q3 = 0.0f;
    mf->ix = 0.0f; mf->iy = 0.0f; mf->iz = 0.0f;
    mf->kp = kp;
    mf->ki = ki;
    mf->dt = sampleTime;
}


void Mahony_Update(MahonyFilter_t* mf,
                   float gx, float gy, float gz,
                   float ax, float ay, float az)
{
    float recipNorm;
    float ex, ey, ez;
    float q0, q1, q2, q3;
    float halfvx, halfvy, halfvz;

    if (mf == NULL) {
    	return;
    }

    gx *= DEG_TO_RAD_F;
    gy *= DEG_TO_RAD_F;
    gz *= DEG_TO_RAD_F;

    recipNorm = ax*ax + ay*ay + az*az;
    if (recipNorm < 1e-6f) goto integrate;
    recipNorm = 1.0f / sqrtf(recipNorm);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    q0 = mf->q0; q1 = mf->q1; q2 = mf->q2; q3 = mf->q3;
    halfvx =  q1*q3 - q0*q2;
    halfvy =  q0*q1 + q2*q3;
    halfvz =  q0*q0 - 0.5f + q3*q3;

    ex = ay*halfvz - az*halfvy;
    ey = az*halfvx - ax*halfvz;
    ez = ax*halfvy - ay*halfvx;

    if (mf->ki > 0.0f) {
        mf->ix += mf->ki * ex * mf->dt;
        mf->iy += mf->ki * ey * mf->dt;
        mf->iz += mf->ki * ez * mf->dt;
        gx += mf->ix;
        gy += mf->iy;
        gz += mf->iz;
    }

    gx += mf->kp * ex;
    gy += mf->kp * ey;
    gz += mf->kp * ez;

integrate:
    {
        float halfdt = 0.5f * mf->dt;
        q0 = mf->q0; q1 = mf->q1; q2 = mf->q2; q3 = mf->q3;
        mf->q0 += (-q1*gx - q2*gy - q3*gz) * halfdt;
        mf->q1 += ( q0*gx + q2*gz - q3*gy) * halfdt;
        mf->q2 += ( q0*gy - q1*gz + q3*gx) * halfdt;
        mf->q3 += ( q0*gz + q1*gy - q2*gx) * halfdt;
    }

    recipNorm = mf->q0*mf->q0 + mf->q1*mf->q1 +
                mf->q2*mf->q2 + mf->q3*mf->q3;
    recipNorm = 1.0f / sqrtf(recipNorm);
    mf->q0 *= recipNorm;
    mf->q1 *= recipNorm;
    mf->q2 *= recipNorm;
    mf->q3 *= recipNorm;
}


void Mahony_GetEuler(const MahonyFilter_t* mf, float* roll, float* pitch, float* yaw) {
    float q0, q1, q2, q3;
    if (mf == NULL || roll == NULL || pitch == NULL || yaw == NULL) {
    	return;
    }

    q0 = mf->q0; q1 = mf->q1; q2 = mf->q2; q3 = mf->q3;

    *roll  = atan2f(2.0f*(q0*q1 + q2*q3),
                    1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG_F;

    *pitch = asinf(2.0f*(q0*q2 - q3*q1)) * RAD_TO_DEG_F;

    *yaw   = atan2f(2.0f*(q0*q3 + q1*q2),
                    1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD_TO_DEG_F;
}


void Mahony_Reset(MahonyFilter_t* mf) {
    if (mf == NULL) {
    	return;
    }
    mf->q0 = 1.0f; mf->q1 = 0.0f; mf->q2 = 0.0f; mf->q3 = 0.0f;
    mf->ix = 0.0f; mf->iy = 0.0f; mf->iz = 0.0f;
}

/*
 * Title: filter.h
 *
 * Date: 17/02/2026
 *
 * Author: Muhammed Selman Çetin
 *
 * GitHub: K4lender
 */


#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/////////////////////////// 2nd Order Low Pass Filter //////////////////////////////////////////
typedef struct {
    float out;        
    float buf[2];    
    float coeffNum;   
    float coeffDen[2];
} LowPassFilter_t;

// Initialize Low Pass filter
void LPF_Init(LowPassFilter_t* lpf, float cutoffFreq, float sampleTime);

// Filter new sample
float LPF_Update(LowPassFilter_t* lpf, float input);

// Reset filter state
void LPF_Reset(LowPassFilter_t* lpf);


//////////////////////////// 2nd Order Notch (Band-Stop) Filter ////////////////////////////////////
typedef struct {
    float alpha;        
    float beta;             
    float x[3];             
    float y[3];             
} NotchFilter_t;

// Initialize Notch filter
void Notch_Init(NotchFilter_t* nf, float centerFreq, float notchWidth, float sampleTime);

// Filter new sample
float Notch_Update(NotchFilter_t* nf, float input);

// Reset filter state
void Notch_Reset(NotchFilter_t* nf);


/////////////////////////// Complementary Filter  ////////////////////////////////////////////
typedef struct {
    float angle;        
    float alpha;        
    float dt;           
} ComplementaryFilter_t;

// Initialize Complementary filter
void CF_Init(ComplementaryFilter_t* cf, float alpha, float sampleTime);

// Update angle estimate
float CF_Update(ComplementaryFilter_t* cf, float gyro_rate, float accel_angle);

// Reset filter state
void CF_Reset(ComplementaryFilter_t* cf);


/////////////////////////// Mahony Filter (Quaternion-based) ////////////////////////////
typedef struct {
    float q0, q1, q2, q3;
    float ix,  iy,  iz;
    float kp;
    float ki;
    float dt;
} MahonyFilter_t;

// Initialize Mahony filter
void Mahony_Init(MahonyFilter_t* mf, float kp, float ki, float sampleTime);

// Update quaternion with gyro (dps) + accelerometer (g)
void Mahony_Update(MahonyFilter_t* mf,
                   float gx, float gy, float gz,
                   float ax, float ay, float az);

// Compute Euler angles from quaternion (degrees)
void Mahony_GetEuler(const MahonyFilter_t* mf,
                     float* roll, float* pitch, float* yaw);

// Reset filter state
void Mahony_Reset(MahonyFilter_t* mf);

#endif /* FILTER_H */

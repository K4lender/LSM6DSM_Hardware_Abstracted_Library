/*
 * Title: lsm6dsm_process.h
 *
 * Date: 20/02/2026
 *
 * Author: Muhammed Selman Çetin
 *
 * GitHub: K4lender
 */


#ifndef LSM6DSM_PROCESS_H
#define LSM6DSM_PROCESS_H

#include "lsm6dsm_driver.h"
#include "filter.h"


//  Default filter parameters
#define PROCESS_LPF_CUTOFF_HZ       20.0f   // Accelerometer + Gyroscope LPF cutoff frequency
#define PROCESS_NOTCH_CENTER_HZ     50.0f   // Gyroscope Notch center frequency (power line EMI)
#define PROCESS_NOTCH_WIDTH_HZ      10.0f   // Gyroscope Notch bandwidth
#define PROCESS_MAHONY_KP           5.0f    // Mahony proportional gain
#define PROCESS_MAHONY_KI           0.001f  // Mahony integral gain (gyro bias correction)
#define PROCESS_CALIB_SAMPLES       200     // Sample count for startup gyro bias calibration


//  Filter configuration
typedef struct {
    float lpf_cutoff_hz;       // Accelerometer and gyroscope LPF cutoff frequency [Hz]
    float notch_center_hz;     // Gyroscope Notch center frequency [Hz]
    float notch_width_hz;      // Gyroscope Notch bandwidth [Hz]
} LSM6DSM_Process_Config_t;


//  Filter struct
typedef struct {
    // Accelerometer 2nd order Bessel LPF
    LowPassFilter_t lpf_ax;
    LowPassFilter_t lpf_ay;
    LowPassFilter_t lpf_az;

    // Gyroscope Notch + LPF
    NotchFilter_t   notch_gx;
    NotchFilter_t   notch_gy;
    NotchFilter_t   notch_gz;
    LowPassFilter_t lpf_gx;
    LowPassFilter_t lpf_gy;
    LowPassFilter_t lpf_gz;

    // Mahony filter
    MahonyFilter_t mahony;

    // Gyro bias
    float bias_gx;
    float bias_gy;
    float bias_gz;
    bool  bias_calibrated;

    bool initialized;
} LSM6DSM_Process_t;


// Configure filter pipeline, perform gyro bias calibration, and
// register the process callback with the driver.
//   get_time_fn : Time source (e.g., HAL_GetTick). If NULL, loop counter is used.
//   user_fn     : Custom filter callback. If NULL, internal LSM6DSM_Process_Callback is used.
LSM6DSM_Status_t LSM6DSM_Process_Init(LSM6DSM_Process_t *proc,
                                       float sample_rate_hz,
                                       const LSM6DSM_Process_Config_t *config,
                                       LSM6DSM_GetTimeFn_t get_time_fn,
                                       LSM6DSM_ProcessFn_t user_fn);

// Reset all filter state.
void LSM6DSM_Process_Reset(LSM6DSM_Process_t *proc);

#endif /* LSM6DSM_PROCESS_H */

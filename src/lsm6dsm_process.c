/*
 * Title: lsm6dsm_process.c
 *
 * Date: 20/02/2026
 *
 * Author: Muhammed Selman Çetin
 *
 * GitHub: K4lender
 */


#include "lsm6dsm_process.h"
#include <string.h>


static LSM6DSM_Status_t Process_CalibrateBias(LSM6DSM_Process_t *proc, LSM6DSM_GetTimeFn_t get_time_fn, uint16_t n_samples);
static LSM6DSM_Status_t LSM6DSM_Process_Callback(const LSM6DSM_Data_t *scaled, LSM6DSM_FilteredData_t *out, void *ctx);


// Default filter parameters (used when config = NULL)
static const LSM6DSM_Process_Config_t s_default_config = {
    PROCESS_LPF_CUTOFF_HZ,
    PROCESS_NOTCH_CENTER_HZ,
    PROCESS_NOTCH_WIDTH_HZ
};


LSM6DSM_Status_t LSM6DSM_Process_Init(LSM6DSM_Process_t *proc,
                                       float sample_rate_hz,
                                       const LSM6DSM_Process_Config_t *config,
                                       LSM6DSM_GetTimeFn_t get_time_fn,
                                       LSM6DSM_ProcessFn_t user_fn) {
    const LSM6DSM_Process_Config_t *cfg;
    float sample_time_s;

    if (proc == NULL) {
        return IMU_ERR_NULL_PTR;
    }

    if (sample_rate_hz <= 0.0f) {
        return IMU_ERR_INVALID_PARAM;
    }

    if(proc->initialized) {
        return IMU_ERR_NOT_READY;  // Already initialized, LSM6DSM_Process_Reset must be called first
    }

    memset(proc, 0, sizeof(*proc));

    cfg = (config != NULL) ? config : &s_default_config;
    sample_time_s  = 1.0f / sample_rate_hz;

    // Accelerometer: 2nd order Bessel LPF
    LPF_Init(&proc->lpf_ax, cfg->lpf_cutoff_hz, sample_time_s);
    LPF_Init(&proc->lpf_ay, cfg->lpf_cutoff_hz, sample_time_s);
    LPF_Init(&proc->lpf_az, cfg->lpf_cutoff_hz, sample_time_s);

    // Gyroscope: Notch + LPF
    Notch_Init(&proc->notch_gx, cfg->notch_center_hz, cfg->notch_width_hz, sample_time_s);
    Notch_Init(&proc->notch_gy, cfg->notch_center_hz, cfg->notch_width_hz, sample_time_s);
    Notch_Init(&proc->notch_gz, cfg->notch_center_hz, cfg->notch_width_hz, sample_time_s);
    LPF_Init(&proc->lpf_gx, cfg->lpf_cutoff_hz, sample_time_s);
    LPF_Init(&proc->lpf_gy, cfg->lpf_cutoff_hz, sample_time_s);
    LPF_Init(&proc->lpf_gz, cfg->lpf_cutoff_hz, sample_time_s);

    // Mahony filter
    Mahony_Init(&proc->mahony, PROCESS_MAHONY_KP, PROCESS_MAHONY_KI, sample_time_s);

    proc->initialized = true;

    // Gyro bias calibration
    Process_CalibrateBias(proc, get_time_fn, PROCESS_CALIB_SAMPLES);

    // Register process callback with driver
    LSM6DSM_RegisterProcessFn((user_fn != NULL) ? user_fn : LSM6DSM_Process_Callback, proc);

    return IMU_OK;
}


void LSM6DSM_Process_Reset(LSM6DSM_Process_t *proc) {
    if (proc == NULL) {
        return;
    }

    LPF_Reset(&proc->lpf_ax);
    LPF_Reset(&proc->lpf_ay);
    LPF_Reset(&proc->lpf_az);

    Notch_Reset(&proc->notch_gx);
    Notch_Reset(&proc->notch_gy);
    Notch_Reset(&proc->notch_gz);
    LPF_Reset(&proc->lpf_gx);
    LPF_Reset(&proc->lpf_gy);
    LPF_Reset(&proc->lpf_gz);

    Mahony_Reset(&proc->mahony);

    proc->bias_gx = 0.0f;
    proc->bias_gy = 0.0f;
    proc->bias_gz = 0.0f;
    proc->bias_calibrated = false;

    // For ReInit
    proc->initialized = false;
}


static LSM6DSM_Status_t Process_CalibrateBias(LSM6DSM_Process_t *proc, LSM6DSM_GetTimeFn_t get_time_fn, uint16_t n_samples) {
    LSM6DSM_Data_t data;
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    uint16_t count = 0;

    if (proc == NULL || !proc->initialized) {
        return IMU_ERR_NULL_PTR;
    }

    while (count < n_samples) {
        if (get_time_fn != NULL) {
            uint32_t t_start = get_time_fn(NULL);
            while (!LSM6DSM_DataReady()) {
                if ((get_time_fn(NULL) - t_start) > 50U) {
                	return IMU_ERR_TIMEOUT;
                }
            }
        } else {
            uint32_t retries = 100000UL;
            while (!LSM6DSM_DataReady()) {
                if (--retries == 0U) {
                	return IMU_ERR_TIMEOUT;
                }
            }
        }
        if (LSM6DSM_ReadScaled(&data) != IMU_OK) {
        	continue;
        }
        sum_gx += data.gyro_x;
        sum_gy += data.gyro_y;
        sum_gz += data.gyro_z;
        count++;
    }

    proc->bias_gx = sum_gx / (float)n_samples;
    proc->bias_gy = sum_gy / (float)n_samples;
    proc->bias_gz = sum_gz / (float)n_samples;
    proc->bias_calibrated = true;

    return IMU_OK;
}


static LSM6DSM_Status_t LSM6DSM_Process_Callback(const LSM6DSM_Data_t *scaled, LSM6DSM_FilteredData_t *out, void *ctx) {
    LSM6DSM_Process_t *proc = (LSM6DSM_Process_t *)ctx;
    float notch_gx, notch_gy, notch_gz;

    if (proc == NULL || scaled == NULL || out == NULL) {
        return IMU_ERR_NULL_PTR;
    }

    if (!proc->initialized) {
        return IMU_ERR_NOT_READY;
    }

    // Accelerometer: 2nd order Bessel LPF
    out->accel_x = LPF_Update(&proc->lpf_ax, scaled->accel_x);
    out->accel_y = LPF_Update(&proc->lpf_ay, scaled->accel_y);
    out->accel_z = LPF_Update(&proc->lpf_az, scaled->accel_z);

    // Jiroskop: First Bias, then Notch and LPF
    {
        float gx_raw = scaled->gyro_x - proc->bias_gx;
        float gy_raw = scaled->gyro_y - proc->bias_gy;
        float gz_raw = scaled->gyro_z - proc->bias_gz;
        notch_gx    = Notch_Update(&proc->notch_gx, gx_raw);
        notch_gy    = Notch_Update(&proc->notch_gy, gy_raw);
        notch_gz    = Notch_Update(&proc->notch_gz, gz_raw);
    }
    out->gyro_x = LPF_Update(&proc->lpf_gx, notch_gx);
    out->gyro_y = LPF_Update(&proc->lpf_gy, notch_gy);
    out->gyro_z = LPF_Update(&proc->lpf_gz, notch_gz);

    // Mahony filter: roll, pitch, yaw
    Mahony_Update(&proc->mahony,
                  out->gyro_x,  out->gyro_y,  out->gyro_z,
                  out->accel_x, out->accel_y, out->accel_z);
    Mahony_GetEuler(&proc->mahony,
                    &out->roll_deg, &out->pitch_deg, &out->yaw_deg);

    // For Temprature
    out->temperature_c = scaled->temp_c;
    out->timestamp_ms  = scaled->timestamp;

    return IMU_OK;
}

/*
 * Title: lsm6dsm_driver.h
 *
 * Date: 17/02/2026
 *
 * Author: Muhammed Selman Çetin
 *
 * GitHub: K4lender
 */


#ifndef LSM6DSM_DRIVER_H
#define LSM6DSM_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "lsm6dsm_defs.h"


// Accelerometer Scales
typedef enum {
    ACCEL_FS_2G  = XL_FS_2G,
    ACCEL_FS_4G  = XL_FS_4G,
    ACCEL_FS_8G  = XL_FS_8G,
    ACCEL_FS_16G = XL_FS_16G,
} AccelScale_t;


// Gyroscope Scales
typedef enum {
    GYRO_FS_125  = 4,
    GYRO_FS_250  = G_FS_250DPS,
    GYRO_FS_500  = G_FS_500DPS,
    GYRO_FS_1000 = G_FS_1000DPS,
    GYRO_FS_2000 = G_FS_2000DPS,
} GyroScale_t;


// Output Data Rate
typedef enum {
    ODR_OFF    = REG_ODR_POWER_DOWN,
    ODR_12_5HZ = REG_ODR_12_5HZ,
    ODR_26HZ   = REG_ODR_26HZ,
    ODR_52HZ   = REG_ODR_52HZ,
    ODR_104HZ  = REG_ODR_104HZ, // Default
    ODR_208HZ  = REG_ODR_208HZ,
    ODR_416HZ  = REG_ODR_416HZ, // High Performance
    ODR_833HZ  = REG_ODR_833HZ,
    ODR_1660HZ = REG_ODR_1660HZ,
    ODR_3330HZ = REG_ODR_3330HZ,
    ODR_6660HZ = REG_ODR_6660HZ,
} OutputDataRate_t;


typedef struct {
    AccelScale_t     accel_scale;
    GyroScale_t      gyro_scale;
    OutputDataRate_t odr;
} LSM6DSM_Config_t;


typedef struct {
    float    accel_x;      // g
    float    accel_y;
    float    accel_z;
    float    gyro_x;       // dps 
    float    gyro_y;
    float    gyro_z;
    float    temp_c;       // Celsius
    uint32_t timestamp;
} LSM6DSM_Data_t;


// Platform I/O callbacks
typedef int32_t  (*LSM6DSM_WriteFn_t)  (void *handle, uint8_t reg, const uint8_t *data, uint16_t len);
typedef int32_t  (*LSM6DSM_ReadFn_t)   (void *handle, uint8_t reg, uint8_t *data, uint16_t len);

// Time source callback (e.g., HAL_GetTick). If NULL, the timestamp from InjectRawData is used.
typedef uint32_t (*LSM6DSM_GetTimeFn_t)(void *handle);

// User filter pipeline callback.
// scaled : physical value scaled by the driver (g, dps, degC)
// out    : filtered output - filled by the user
// ctx    : user context (can be NULL)
typedef LSM6DSM_Status_t (*LSM6DSM_ProcessFn_t)(const LSM6DSM_Data_t *scaled, LSM6DSM_FilteredData_t *out, void *ctx);


// Configure and initialize the sensor driver.
// In simulation mode, write_fn, read_fn, get_time_fn are passed as NULL.
LSM6DSM_Status_t LSM6DSM_Init(LSM6DSM_WriteFn_t  write_fn,
                           LSM6DSM_ReadFn_t   read_fn,
                           LSM6DSM_GetTimeFn_t get_time_fn,
                           void              *handle,
                           LSM6DSM_Config_t  *config);

// Shut down the sensor
void LSM6DSM_DeInit(void);

// Check the sensor's WHO_AM_I register
bool LSM6DSM_CheckID(void);

// Read raw data
LSM6DSM_Status_t LSM6DSM_ReadRaw(LSM6DSM_RawData_t *data);

// Read data converted to physical units
LSM6DSM_Status_t LSM6DSM_ReadScaled(LSM6DSM_Data_t *data);

// Check if data is ready
bool LSM6DSM_DataReady(void);

// Inject raw data from simulation layer into driver's register table. (Simulation mode only)
LSM6DSM_Status_t LSM6DSM_InjectRawData(const LSM6DSM_RawData_t *data);

// Register user filter pipeline.
// process_fn : filter callback (NULL = filtering disabled)
// ctx        : user context passed to callback
void LSM6DSM_RegisterProcessFn(LSM6DSM_ProcessFn_t process_fn, void *ctx);

// Read filtered data via the internal process_fn callback.
// A callback must be registered via LSM6DSM_RegisterProcessFn.
LSM6DSM_Status_t LSM6DSM_ReadProcessed(LSM6DSM_FilteredData_t *data);

// Returns sampling rate in Hz
float LSM6DSM_GetSampleRate(void);

// FIFO configuration
typedef struct {
    uint8_t  odr;           // One of FIFO_ODR_* constants (must be equal to or less than sensor ODR)
    uint8_t  mode;          // One of FIFO_MODE_* constants
    uint16_t threshold;     // Watermark threshold [words, 0-2047] 0 = watermark interrupt disabled
    uint8_t  gyro_dec;      // Gyroscope decimation  - from FIFO_DEC_* constants
    uint8_t  accel_dec;     // Accelerometer decimation - from FIFO_DEC_* constants
} LSM6DSM_FifoConfig_t;

// FIFO status information
typedef struct {
    uint16_t word_count;    // Number of 16-bit words in FIFO
    uint16_t sample_count;  // Number of complete IMU samples ready (word_count / 6)
    bool     watermark;     // Watermark threshold exceeded (WTM bit)
    bool     overrun;       // Overflow: data loss occurred (OVR bit)
    bool     full;          // FIFO completely full
    bool     empty;         // FIFO empty
} LSM6DSM_FifoStatus_t;

// Configure and start the FIFO. Must be called AFTER LSM6DSM_Init(). If config = NULL: 104 Hz, CONT mode, WM=200, DEC=/1
LSM6DSM_Status_t LSM6DSM_FIFO_Init(const LSM6DSM_FifoConfig_t *config);

// Stop FIFO (sets to Bypass mode, content is cleared).
void LSM6DSM_FIFO_Stop(void);

// Read FIFO status registers.
LSM6DSM_Status_t LSM6DSM_FIFO_GetStatus(LSM6DSM_FifoStatus_t *status);

// Read a single IMU sample from FIFO [Gx Gy Gz Ax Ay Az = 6 words]. Returns IMU_ERR_BUFFER_EMPTY if not enough data.
LSM6DSM_Status_t LSM6DSM_FIFO_ReadSample(LSM6DSM_RawData_t *out);

// Perform bulk read from FIFO.
// buf       : LSM6DSM_RawData_t array to fill
// max_count : Capacity of the array
// read_count: Actual number of samples read (can be NULL)
LSM6DSM_Status_t LSM6DSM_FIFO_ReadBurst(LSM6DSM_RawData_t *buf, uint16_t max_count, uint16_t *read_count);

// Flush FIFO content (discards data).
LSM6DSM_Status_t LSM6DSM_FIFO_Flush(void);

// Returns how many complete IMU samples are available to read.
uint16_t LSM6DSM_FIFO_GetAvailableSamples(void);

#endif /* LSM6DSM_DRIVER_H */

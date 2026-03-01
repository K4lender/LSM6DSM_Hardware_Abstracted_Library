/*
 * Title: lsm6dsm_defs.h
 *
 * Date: 17/02/2026
 *
 * Author: Muhammed Selman Çetin
 *
 * GitHub: K4lender
 */


#ifndef LSM6DSM_DEFS_H
#define LSM6DSM_DEFS_H

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/* 1. GENERAL IMU TYPES AND STATUS CODES                                       */
/* ========================================================================== */

#define IMU_BUFFER_SIZE         128 


typedef enum {
    IMU_OK = 0,             
    IMU_ERR_COMM,              
    IMU_ERR_TIMEOUT,            
    IMU_ERR_INVALID_ID,         
    IMU_ERR_NOT_READY,          
    IMU_ERR_BUFFER_FULL,        
    IMU_ERR_BUFFER_EMPTY,       
    IMU_ERR_NULL_PTR,           
    IMU_ERR_INVALID_PARAM       
} LSM6DSM_Status_t;


typedef struct {
    int16_t accel_x;            // X-axis acceleration (raw)
    int16_t accel_y;            // Y-axis acceleration (raw)
    int16_t accel_z;            // Z-axis acceleration (raw)
    int16_t gyro_x;             // X-axis angular rate (raw)
    int16_t gyro_y;             // Y-axis angular rate (raw)
    int16_t gyro_z;             // Z-axis angular rate (raw)
    int16_t temperature;        // Temperature (raw)
    uint32_t timestamp_ms;      // Timestamp (ms)
} LSM6DSM_RawData_t;


// Filtered IMU data (scaling + filtering applied during read)
typedef struct {
    float accel_x;              // X-axis acceleration (g)
    float accel_y;              // Y-axis acceleration (g)
    float accel_z;              // Z-axis acceleration (g)
    float gyro_x;               // X-axis angular rate (dps)
    float gyro_y;               // Y-axis angular rate (dps)
    float gyro_z;               // Z-axis angular rate (dps)
    float roll_deg;             // Roll angle - X axis (degrees)
    float pitch_deg;            // Pitch angle - Y axis (degrees)
    float yaw_deg;              // Yaw angle - Z axis, gyro only (degrees)
    float temperature_c;        // Temperature (°C)
    uint32_t timestamp_ms;      // Timestamp (ms)
} LSM6DSM_FilteredData_t;

// Accelerometer sensitivity [mg/LSB]
#define SENS_2G_MG_PER_LSB      0.061f
#define SENS_4G_MG_PER_LSB      0.122f
#define SENS_8G_MG_PER_LSB      0.244f
#define SENS_16G_MG_PER_LSB     0.488f

// Gyroscope sensitivity [mdps/LSB]
#define SENS_125DPS_MDPS_PER_LSB    4.375f
#define SENS_250DPS_MDPS_PER_LSB    8.75f
#define SENS_500DPS_MDPS_PER_LSB    17.50f
#define SENS_1000DPS_MDPS_PER_LSB   35.00f
#define SENS_2000DPS_MDPS_PER_LSB   70.00f

// ODR Hz values
#define ODR_HZ_OFF              0.0f
#define ODR_HZ_12_5             12.5f
#define ODR_HZ_26               26.0f
#define ODR_HZ_52               52.0f
#define ODR_HZ_104              104.0f
#define ODR_HZ_208              208.0f
#define ODR_HZ_416              416.0f
#define ODR_HZ_833              833.0f
#define ODR_HZ_1660             1660.0f
#define ODR_HZ_3330             3330.0f
#define ODR_HZ_6660             6660.0f

#define IMU_TEMP_RAW_TO_CELSIUS(raw)    (((float)(raw) / 256.0f) + 25.0f)

/* ========================================================================== */
/* 3. LSM6DSM REGISTER MAP (LSM6DSM_REGS)                                      */
/* ========================================================================== */

// I2C address (SDO = GND)
#define LSM6DSM_I2C_ADDR_LOW    0x6A
#define LSM6DSM_I2C_ADDR_HIGH   0x6B    // If SDO = VDD

// Device ID
#define LSM6DSM_REG_WHO_AM_I        0x0F
#define LSM6DSM_WHO_AM_I_VAL        0x6A

// Control registers
#define LSM6DSM_REG_CTRL1_XL        0x10    // Accelerometer control
#define LSM6DSM_REG_CTRL2_G         0x11    // Gyroscope control
#define LSM6DSM_REG_CTRL3_C         0x12    // System control

// Status register
#define LSM6DSM_REG_STATUS          0x1E

// Temperature output registers
#define LSM6DSM_REG_OUT_TEMP_L      0x20
#define LSM6DSM_REG_OUT_TEMP_H      0x21

// Gyroscope output registers
#define LSM6DSM_REG_OUTX_L_G        0x22
#define LSM6DSM_REG_OUTX_H_G        0x23
#define LSM6DSM_REG_OUTY_L_G        0x24
#define LSM6DSM_REG_OUTY_H_G        0x25
#define LSM6DSM_REG_OUTZ_L_G        0x26
#define LSM6DSM_REG_OUTZ_H_G        0x27

// Accelerometer output registers
#define LSM6DSM_REG_OUTX_L_XL       0x28
#define LSM6DSM_REG_OUTX_H_XL       0x29
#define LSM6DSM_REG_OUTY_L_XL       0x2A
#define LSM6DSM_REG_OUTY_H_XL       0x2B
#define LSM6DSM_REG_OUTZ_L_XL       0x2C
#define LSM6DSM_REG_OUTZ_H_XL       0x2D

// CTRL1_XL (0x10) bit fields
// [7:4] ODR_XL  |  [3:2] FS_XL  |  [1] LPF1_BW_SEL  |  [0] BW0_XL
#define CTRL1_XL_ODR_SHIFT          4
#define CTRL1_XL_FS_SHIFT           2

// FS_XL encoding
#define XL_FS_2G    0x00
#define XL_FS_16G   0x01    // 01 = +/-16g
#define XL_FS_4G    0x02
#define XL_FS_8G    0x03

// CTRL2_G (0x11) bit fields
// [7:4] ODR_G  |  [3:2] FS_G  |  [1] FS_125  |  [0] 0 (reserved)
#define CTRL2_G_ODR_SHIFT           4
#define CTRL2_G_FS_SHIFT            2
#define CTRL2_G_FS125_BIT           0x02    // enables +/-125 dps

// FS_G encoding
#define G_FS_250DPS     0x00
#define G_FS_500DPS     0x01
#define G_FS_1000DPS    0x02
#define G_FS_2000DPS    0x03

// ODR register encoding 
#define REG_ODR_POWER_DOWN  0x00
#define REG_ODR_12_5HZ      0x01
#define REG_ODR_26HZ        0x02
#define REG_ODR_52HZ        0x03
#define REG_ODR_104HZ       0x04
#define REG_ODR_208HZ       0x05
#define REG_ODR_416HZ       0x06
#define REG_ODR_833HZ       0x07
#define REG_ODR_1660HZ      0x08
#define REG_ODR_3330HZ      0x09
#define REG_ODR_6660HZ      0x0A

// CTRL3_C (0x12) bit fields
// [7] BOOT | [6] BDU | [5] H_LACTIVE | [4] PP_OD [3] SIM  | [2] IF_INC | [1] BLE | [0] SW_RESET
#define CTRL3_C_BOOT        0x80    // Reload memory content
#define CTRL3_C_BDU         0x40    // Block Data Update 
#define CTRL3_C_IF_INC      0x04    // Automatic address increment (burst read)
#define CTRL3_C_SW_RESET    0x01    // Software reset

// STATUS_REG (0x1E) bit fields
#define STATUS_TDA      0x04    // Temperature data ready
#define STATUS_GDA      0x02    // Gyroscope data ready
#define STATUS_XLDA     0x01    // Accelerometer data ready

// FIFO Kontrol Registerlari
// LSM6DSM hardware FIFO capacity is 4096 bytes (2048 x 16-bit words). Maximum 2048 / 6 = 341 complete IMU samples can be stored.
// When both Accel + Gyro are active, 6 words (12 bytes) are written per sample [Gx][Gy][Gz][Ax][Ay][Az]
#define LSM6DSM_REG_FIFO_CTRL1      0x06    // WTM[7:0]          - threshold low byte
#define LSM6DSM_REG_FIFO_CTRL2      0x07    // WTM[10:8]         - threshold high bit
#define LSM6DSM_REG_FIFO_CTRL3      0x08    // DEC_GYRO | DEC_XL - decimation
#define LSM6DSM_REG_FIFO_CTRL4      0x09    // STOP_ON_FTH       - threshold protection
#define LSM6DSM_REG_FIFO_CTRL5      0x0A    // ODR_FIFO | FIFO_MODE

#define LSM6DSM_REG_FIFO_STATUS1    0x3A    // DIFF_FIFO[7:0]  - fill count low byte
#define LSM6DSM_REG_FIFO_STATUS2    0x3B    // WTM|OVR|FULL|EMPTY|DIFF[10:8]
#define LSM6DSM_REG_FIFO_STATUS3    0x3C    // FIFO_PATTERN[7:0]
#define LSM6DSM_REG_FIFO_STATUS4    0x3D    // FIFO_PATTERN[9:8]

#define LSM6DSM_REG_FIFO_DATA_OUT_L 0x3E   // Read register (low byte)
#define LSM6DSM_REG_FIFO_DATA_OUT_H 0x3F   // Read register (high byte)

// FIFO_CTRL3 (0x08) - Decimation
// Gyro [5:3] | Accel [2:0] 0x00 -> sensor not written to FIFO 0x01 -> every sample written (/1)
#define FIFO_DEC_DISABLED   0x00
#define FIFO_DEC_1          0x01
#define FIFO_DEC_2          0x02
#define FIFO_DEC_3          0x03
#define FIFO_DEC_4          0x04
#define FIFO_DEC_8          0x05
#define FIFO_DEC_16         0x06
#define FIFO_DEC_32         0x07

#define FIFO_CTRL3_GYRO_SHIFT   3
#define FIFO_CTRL3_XL_SHIFT     0

// ODR encoding
#define FIFO_ODR_DISABLED   0x00
#define FIFO_ODR_12_5HZ     0x08
#define FIFO_ODR_26HZ       0x10
#define FIFO_ODR_52HZ       0x18
#define FIFO_ODR_104HZ      0x20
#define FIFO_ODR_208HZ      0x28
#define FIFO_ODR_416HZ      0x30
#define FIFO_ODR_833HZ      0x38
#define FIFO_ODR_1660HZ     0x40
#define FIFO_ODR_3330HZ     0x48
#define FIFO_ODR_6660HZ     0x50

// FIFO mode encoding
#define FIFO_MODE_BYPASS     0x00    // FIFO disabled, content cleared
#define FIFO_MODE_STOP       0x01    // Reject new data when full
#define FIFO_MODE_CONT_UTD   0x03    // Switch to Continuous mode when threshold exceeded
#define FIFO_MODE_BYPASS_UTD 0x04   // Switch to Bypass mode when threshold exceeded
#define FIFO_MODE_CONT       0x06    // Continuous: overwrite oldest data when full

// FIFO_STATUS2 (0x3B) - Status bits
#define FIFO_STATUS2_WTM        0x80    // Watermark: threshold exceeded
#define FIFO_STATUS2_OVERRUN    0x40    // Overflow: data loss occurred
#define FIFO_STATUS2_FULL       0x20    // FIFO completely full
#define FIFO_STATUS2_EMPTY      0x10    // FIFO empty
#define FIFO_STATUS2_DIFF_MASK  0x07    // DIFF_FIFO[10:8] - upper 3 bits

#endif /* LSM6DSM_DEFS_H */

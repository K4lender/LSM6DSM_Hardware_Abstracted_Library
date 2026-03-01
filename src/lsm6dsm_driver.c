/*
 * Title: lsm6dsm_driver.c
 *
 * Date: 17/02/2026
 *
 * Author: Muhammed Selman Çetin
 *
 * GitHub: K4lender
 */


#include "lsm6dsm_driver.h"
#include <string.h>


#define REG_TABLE_SIZE  0x40
#define ODR_TABLE_LEN   (sizeof(s_odr_hz) / sizeof(s_odr_hz[0]))
#define FIFO_WORDS_PER_SAMPLE   6U

static uint8_t s_reg_table[REG_TABLE_SIZE];


static const float s_accel_sens[] = {
    SENS_2G_MG_PER_LSB,         // 2G:  0.061 mg/LSB     XL_FS_2G  = 0
    SENS_16G_MG_PER_LSB,        // 16G: 0.488 mg/LSB     XL_FS_16G = 1
    SENS_4G_MG_PER_LSB,         // 4G:  0.122 mg/LSB     XL_FS_4G  = 2
    SENS_8G_MG_PER_LSB,         // 8G:  0.244 mg/LSB     XL_FS_8G  = 3
};


static const float s_gyro_sens[] = {
    SENS_250DPS_MDPS_PER_LSB,   // 250 dps:  8.75 mdps/LSB      GYRO_FS_250  = 0
    SENS_500DPS_MDPS_PER_LSB,   // 500 dps:  17.50 mdps/LSB     GYRO_FS_500  = 1
    SENS_1000DPS_MDPS_PER_LSB,  // 1000 dps: 35.00 mdps/LSB     GYRO_FS_1000 = 2
    SENS_2000DPS_MDPS_PER_LSB,  // 2000 dps: 70.00 mdps/LSB     GYRO_FS_2000 = 3
    SENS_125DPS_MDPS_PER_LSB,   // 125 dps:  4.375 mdps/LSB     GYRO_FS_125  = 4
};


static const float s_odr_hz[] = {
    ODR_HZ_OFF,     // ODR_OFF
    ODR_HZ_12_5,    // ODR_12_5HZ
    ODR_HZ_26,      // ODR_26HZ
    ODR_HZ_52,      // ODR_52HZ
    ODR_HZ_104,     // ODR_104HZ - Default
    ODR_HZ_208,     // ODR_208HZ
    ODR_HZ_416,     // ODR_416HZ - High Performance
    ODR_HZ_833,     // ODR_833HZ
    ODR_HZ_1660,    // ODR_1660HZ
    ODR_HZ_3330,    // ODR_3330HZ
    ODR_HZ_6660     // ODR_6660HZ
};


static struct {
    bool initialized;
    AccelScale_t accel_scale;
    GyroScale_t gyro_scale;
    OutputDataRate_t odr;
    float accel_sensitivity;
    float gyro_sensitivity;
    LSM6DSM_WriteFn_t   write;
    LSM6DSM_ReadFn_t    read;
    LSM6DSM_GetTimeFn_t get_time;
    void               *handle;
    uint32_t            last_timestamp;     // timestamp received via InjectRawData (For Simulation mode)
    LSM6DSM_ProcessFn_t process_fn;
    void               *process_ctx;
    bool                fifo_running;
} s_driver = {0};

// If write/read is NULL, accesses simulation table; otherwise calls platform callback.
static void WriteReg(uint8_t reg, uint8_t val) {
    if (s_driver.write != NULL) {
        s_driver.write(s_driver.handle, reg, &val, 1);
    } 
    else if (reg < REG_TABLE_SIZE) {
        s_reg_table[reg] = val;
    }
}


static uint8_t ReadReg(uint8_t reg) {
    uint8_t val = 0;
    if (s_driver.read != NULL) {
        s_driver.read(s_driver.handle, reg, &val, 1);
    } 
    else if (reg < REG_TABLE_SIZE) {
        val = s_reg_table[reg];
    }
    return val;
}


static void ReadRegBurst(uint8_t start_reg, uint8_t *buf, uint16_t len) {
    uint16_t i;
    if (s_driver.read != NULL) {
        s_driver.read(s_driver.handle, start_reg, buf, len);
    } 
    else {
        for (i = 0; i < len; i++) {
            buf[i] = (start_reg + i < REG_TABLE_SIZE) ? s_reg_table[start_reg + i] : 0;
        }
    }
}


static int16_t ToInt16(uint8_t lsb, uint8_t msb) {
    return (int16_t)((uint16_t)msb << 8 | lsb);
}


static void InitRegisterTable(void) {
    memset((void*)s_reg_table, 0, sizeof(s_reg_table));
    s_reg_table[LSM6DSM_REG_WHO_AM_I] = LSM6DSM_WHO_AM_I_VAL;
}


LSM6DSM_Status_t LSM6DSM_Init(LSM6DSM_WriteFn_t  write_fn,
                           LSM6DSM_ReadFn_t   read_fn,
                           LSM6DSM_GetTimeFn_t get_time_fn,
                           void              *handle,
                           LSM6DSM_Config_t  *config) {
    uint8_t ctrl1_xl, ctrl2_g, ctrl3_c;
    uint8_t odr_val;

    // Register platform I/O
    s_driver.write    = write_fn;
    s_driver.read     = read_fn;
    s_driver.get_time = get_time_fn;
    s_driver.handle   = handle;
    
    // Reset register table in simulation mode
    if (write_fn == NULL) {
        InitRegisterTable();
    }

    // Default or user settings
    if (config != NULL) {
        s_driver.accel_scale = config->accel_scale;
        s_driver.gyro_scale  = config->gyro_scale;
        s_driver.odr         = config->odr;
    } else {
        s_driver.accel_scale = ACCEL_FS_2G;
        s_driver.gyro_scale  = GYRO_FS_250;
        s_driver.odr         = ODR_104HZ;
    }
    
    // CTRL3_C: Enable BDU and IF_INC (data consistency + burst read)
    ctrl3_c = CTRL3_C_BDU | CTRL3_C_IF_INC;
    WriteReg(LSM6DSM_REG_CTRL3_C, ctrl3_c);

    // CTRL1_XL: Accelerometer ODR + FS
    odr_val  = (uint8_t)s_driver.odr;
    ctrl1_xl = (uint8_t)((odr_val << CTRL1_XL_ODR_SHIFT) | ((uint8_t)s_driver.accel_scale << CTRL1_XL_FS_SHIFT));
    WriteReg(LSM6DSM_REG_CTRL1_XL, ctrl1_xl);

    // CTRL2_G: Gyroscope ODR + FS
    if (s_driver.gyro_scale == GYRO_FS_125) {
        // Set FS_125 bit, FS_G = 00
        ctrl2_g = (uint8_t)((odr_val << CTRL2_G_ODR_SHIFT) | CTRL2_G_FS125_BIT);
    } else {
        ctrl2_g = (uint8_t)((odr_val << CTRL2_G_ODR_SHIFT) | ((uint8_t)s_driver.gyro_scale  << CTRL2_G_FS_SHIFT));
    }
    WriteReg(LSM6DSM_REG_CTRL2_G, ctrl2_g);
    
    // Set sensitivities
    s_driver.accel_sensitivity = s_accel_sens[(uint8_t)s_driver.accel_scale];
    s_driver.gyro_sensitivity = s_gyro_sens[(uint8_t)s_driver.gyro_scale];

    s_driver.initialized = true;
    return IMU_OK;
}


void LSM6DSM_DeInit(void) {
    WriteReg(LSM6DSM_REG_CTRL1_XL, 0x00);
    WriteReg(LSM6DSM_REG_CTRL2_G,  0x00);
    s_driver.initialized = false;
}


bool LSM6DSM_CheckID(void) {
    return (ReadReg(LSM6DSM_REG_WHO_AM_I) == LSM6DSM_WHO_AM_I_VAL);
}


bool LSM6DSM_DataReady(void) {
    return (ReadReg(LSM6DSM_REG_STATUS) & 0x03) != 0;
}


LSM6DSM_Status_t LSM6DSM_ReadRaw(LSM6DSM_RawData_t* data) {
    uint8_t buf[6];
    
    if (data == NULL) {
        return IMU_ERR_NULL_PTR;
    }
    
    if (!s_driver.initialized) {
        return IMU_ERR_NOT_READY;
    }
    
    // Accelerometer 0x28-0x2D (6 bytes)
    ReadRegBurst(LSM6DSM_REG_OUTX_L_XL, buf, 6);
    data->accel_x = ToInt16(buf[0], buf[1]);
    data->accel_y = ToInt16(buf[2], buf[3]);
    data->accel_z = ToInt16(buf[4], buf[5]);
    
    // Gyroscope 0x22-0x27
    ReadRegBurst(LSM6DSM_REG_OUTX_L_G, buf, 6);
    data->gyro_x = ToInt16(buf[0], buf[1]);
    data->gyro_y = ToInt16(buf[2], buf[3]);
    data->gyro_z = ToInt16(buf[4], buf[5]);
    
    // Temperature 0x20-0x21
    ReadRegBurst(LSM6DSM_REG_OUT_TEMP_L, buf, 2);
    data->temperature = ToInt16(buf[0], buf[1]);
    
    data->timestamp_ms = (s_driver.get_time != NULL) ? s_driver.get_time(s_driver.handle) : s_driver.last_timestamp;
    
    return IMU_OK;
}


LSM6DSM_Status_t LSM6DSM_ReadScaled(LSM6DSM_Data_t* data) {
    LSM6DSM_RawData_t imu_raw;
    LSM6DSM_Status_t imu_st;
    
    if (data == NULL) {
        return IMU_ERR_NULL_PTR;
    }
    
    imu_st = LSM6DSM_ReadRaw(&imu_raw);
    if (imu_st != IMU_OK) {
        return imu_st;
    }
    
    // Raw-to-physical unit conversion
    data->accel_x = (float)imu_raw.accel_x * s_driver.accel_sensitivity / 1000.0f;
    data->accel_y = (float)imu_raw.accel_y * s_driver.accel_sensitivity / 1000.0f;
    data->accel_z = (float)imu_raw.accel_z * s_driver.accel_sensitivity / 1000.0f;
    
    data->gyro_x = (float)imu_raw.gyro_x * s_driver.gyro_sensitivity / 1000.0f;
    data->gyro_y = (float)imu_raw.gyro_y * s_driver.gyro_sensitivity / 1000.0f;
    data->gyro_z = (float)imu_raw.gyro_z * s_driver.gyro_sensitivity / 1000.0f;
    
    data->temp_c = IMU_TEMP_RAW_TO_CELSIUS(imu_raw.temperature);
    
    data->timestamp = imu_raw.timestamp_ms;
    
    return IMU_OK;
}


float LSM6DSM_GetSampleRate(void) {
    if (!s_driver.initialized) {
        return ODR_HZ_104;
    }
    if ((uint8_t)s_driver.odr >= ODR_TABLE_LEN) {
        return ODR_HZ_104;
    }
    return s_odr_hz[s_driver.odr];
}


LSM6DSM_Status_t LSM6DSM_InjectRawData(const LSM6DSM_RawData_t *data) {
    if (data == NULL) {
        return IMU_ERR_NULL_PTR;
    }
    if (!s_driver.initialized) {
        return IMU_ERR_NOT_READY;
    }  

    // Store timestamp (used by ReadRaw() if get_time is NULL)
    s_driver.last_timestamp = data->timestamp_ms;

    // Write to register table in Little-Endian format
    WriteReg(LSM6DSM_REG_OUTX_L_XL, (uint8_t)( data->accel_x        & 0xFF));
    WriteReg(LSM6DSM_REG_OUTX_H_XL, (uint8_t)((data->accel_x >> 8)  & 0xFF));
    WriteReg(LSM6DSM_REG_OUTY_L_XL, (uint8_t)( data->accel_y        & 0xFF));
    WriteReg(LSM6DSM_REG_OUTY_H_XL, (uint8_t)((data->accel_y >> 8)  & 0xFF));
    WriteReg(LSM6DSM_REG_OUTZ_L_XL, (uint8_t)( data->accel_z        & 0xFF));
    WriteReg(LSM6DSM_REG_OUTZ_H_XL, (uint8_t)((data->accel_z >> 8)  & 0xFF));

    WriteReg(LSM6DSM_REG_OUTX_L_G,  (uint8_t)( data->gyro_x         & 0xFF));
    WriteReg(LSM6DSM_REG_OUTX_H_G,  (uint8_t)((data->gyro_x >> 8)   & 0xFF));
    WriteReg(LSM6DSM_REG_OUTY_L_G,  (uint8_t)( data->gyro_y         & 0xFF));
    WriteReg(LSM6DSM_REG_OUTY_H_G,  (uint8_t)((data->gyro_y >> 8)   & 0xFF));
    WriteReg(LSM6DSM_REG_OUTZ_L_G,  (uint8_t)( data->gyro_z         & 0xFF));
    WriteReg(LSM6DSM_REG_OUTZ_H_G,  (uint8_t)((data->gyro_z >> 8)   & 0xFF));

    WriteReg(LSM6DSM_REG_OUT_TEMP_L, (uint8_t)( data->temperature       & 0xFF));
    WriteReg(LSM6DSM_REG_OUT_TEMP_H, (uint8_t)((data->temperature >> 8) & 0xFF));

    WriteReg(LSM6DSM_REG_STATUS, STATUS_TDA | STATUS_GDA | STATUS_XLDA);
    return IMU_OK;
}


void LSM6DSM_RegisterProcessFn(LSM6DSM_ProcessFn_t process_fn, void *ctx) {
    s_driver.process_fn  = process_fn;
    s_driver.process_ctx = ctx;
}


LSM6DSM_Status_t LSM6DSM_ReadProcessed(LSM6DSM_FilteredData_t *data) {
    LSM6DSM_Data_t scaled;
    LSM6DSM_Status_t   imu_st;

    if (data == NULL) {
        return IMU_ERR_NULL_PTR;
    }
    if (s_driver.process_fn == NULL) {
        return IMU_ERR_NOT_READY;
    }

    imu_st = LSM6DSM_ReadScaled(&scaled);
    if (imu_st != IMU_OK) {
        return imu_st;
    }

    return s_driver.process_fn(&scaled, data, s_driver.process_ctx);
}


static LSM6DSM_Status_t FIFO_ReadWord(int16_t *word) {
    uint8_t buf[2];
    ReadRegBurst(LSM6DSM_REG_FIFO_DATA_OUT_L, buf, 2);
    *word = ToInt16(buf[0], buf[1]);
    return IMU_OK;
}


LSM6DSM_Status_t LSM6DSM_FIFO_Init(const LSM6DSM_FifoConfig_t *config) {
    uint8_t  ctrl3, ctrl5;
    uint16_t threshold;
    uint8_t  th_low, th_high;
    uint8_t  gyro_dec, accel_dec;
    uint8_t  odr, mode;

    if (!s_driver.initialized) {
        return IMU_ERR_NOT_READY;
    }

    // First Bypass: clear and stop FIFO
    WriteReg(LSM6DSM_REG_FIFO_CTRL5, FIFO_MODE_BYPASS);

    if (config == NULL) {
        // Default: 104 Hz, Continuous, WM=200, DEC=/1
        threshold = 200;
        gyro_dec  = FIFO_DEC_1;
        accel_dec = FIFO_DEC_1;
        odr       = FIFO_ODR_104HZ;
        mode      = FIFO_MODE_CONT;
    } else {
        threshold = config->threshold;
        gyro_dec  = config->gyro_dec;
        accel_dec = config->accel_dec;
        odr       = config->odr;
        mode      = config->mode;
    }

    // FIFO_CTRL1/2: Watermark threshold (11-bit)
    th_low  = (uint8_t)( threshold       & 0xFF);
    th_high = (uint8_t)((threshold >> 8) & 0x07);
    WriteReg(LSM6DSM_REG_FIFO_CTRL1, th_low);
    WriteReg(LSM6DSM_REG_FIFO_CTRL2, th_high);

    // FIFO_CTRL3: Decimation [5:3]=gyro, [2:0]=accel
    ctrl3 = (uint8_t)(((gyro_dec & 0x07) << FIFO_CTRL3_GYRO_SHIFT) | (accel_dec & 0x07));
    WriteReg(LSM6DSM_REG_FIFO_CTRL3, ctrl3);

    // FIFO_CTRL4: Not used (dataset 3-4 decimation)
    WriteReg(LSM6DSM_REG_FIFO_CTRL4, 0x00);

    // FIFO_CTRL5: ODR | MODE
    ctrl5 = (uint8_t)(odr | mode);
    WriteReg(LSM6DSM_REG_FIFO_CTRL5, ctrl5);

    s_driver.fifo_running = true;
    return IMU_OK;
}


void LSM6DSM_FIFO_Stop(void) {
    WriteReg(LSM6DSM_REG_FIFO_CTRL5, FIFO_MODE_BYPASS);
    s_driver.fifo_running = false;
}


LSM6DSM_Status_t LSM6DSM_FIFO_GetStatus(LSM6DSM_FifoStatus_t *fifo_st) {
    uint8_t  s1, s2;
    uint16_t words;

    if (fifo_st == NULL) {
        return IMU_ERR_NULL_PTR;
    }

    // FIFO_STATUS1: DIFF_FIFO[7:0]   FIFO_STATUS2: WTM|OVR|FULL|EMPTY|DIFF[10:8]
    s1 = ReadReg(LSM6DSM_REG_FIFO_STATUS1);
    s2 = ReadReg(LSM6DSM_REG_FIFO_STATUS2);

    words = (uint16_t)s1 | ((uint16_t)(s2 & FIFO_STATUS2_DIFF_MASK) << 8);

    fifo_st->word_count   = words;
    fifo_st->sample_count = words / FIFO_WORDS_PER_SAMPLE;
    fifo_st->watermark    = (s2 & FIFO_STATUS2_WTM)     != 0;
    fifo_st->overrun      = (s2 & FIFO_STATUS2_OVERRUN)  != 0;
    fifo_st->full         = (s2 & FIFO_STATUS2_FULL)     != 0;
    fifo_st->empty        = (s2 & FIFO_STATUS2_EMPTY)    != 0;

    return IMU_OK;
}


LSM6DSM_Status_t LSM6DSM_FIFO_ReadSample(LSM6DSM_RawData_t *out) {
    int16_t words[FIFO_WORDS_PER_SAMPLE];
    uint8_t i;

    if (out == NULL) {
        return IMU_ERR_NULL_PTR;
    }
    if (!s_driver.fifo_running) {
        return IMU_ERR_NOT_READY;
    }

    // Read 6 words [Gx Gy Gz Ax Ay Az]
    for (i = 0; i < FIFO_WORDS_PER_SAMPLE; i++) {
        if (FIFO_ReadWord(&words[i]) != IMU_OK) {
            return IMU_ERR_COMM;
        }
    }

    out->gyro_x      = words[0];
    out->gyro_y      = words[1];
    out->gyro_z      = words[2];
    out->accel_x     = words[3];
    out->accel_y     = words[4];
    out->accel_z     = words[5];
    out->temperature = 0;              // FIFO does not record temperature

    // Time source: use get_time callback if available, otherwise use last injected timestamp
    out->timestamp_ms = (s_driver.get_time != NULL) ? s_driver.get_time(s_driver.handle) : s_driver.last_timestamp;

    return IMU_OK;
}


LSM6DSM_Status_t LSM6DSM_FIFO_ReadBurst(LSM6DSM_RawData_t *buf, uint16_t max_count, uint16_t *read_count) {
    LSM6DSM_FifoStatus_t fifo_st;
    uint16_t to_read, i;
    LSM6DSM_Status_t imu_st;

    if (buf == NULL) {
        return IMU_ERR_NULL_PTR;
    }            
    if (max_count == 0) {
        return IMU_ERR_INVALID_PARAM;
    }
    if (!s_driver.fifo_running) {
        return IMU_ERR_NOT_READY;
    }

    imu_st = LSM6DSM_FIFO_GetStatus(&fifo_st);
    if (imu_st != IMU_OK) {
        return imu_st;
    }

    to_read = (fifo_st.sample_count < max_count) ? fifo_st.sample_count : max_count;

    for (i = 0; i < to_read; i++) {
        imu_st = LSM6DSM_FIFO_ReadSample(&buf[i]);
        if (imu_st != IMU_OK) {
            break;
        }
    }

    if (read_count != NULL) {
        *read_count = i;
    }

    if (i > 0) {
        return IMU_OK;
    } else {
        return IMU_ERR_BUFFER_EMPTY;
    }
}


LSM6DSM_Status_t LSM6DSM_FIFO_Flush(void) {
    LSM6DSM_FifoStatus_t fifo_st;
    int16_t dummy;

    // Read and discard words until empty
    do {
        if (LSM6DSM_FIFO_GetStatus(&fifo_st) != IMU_OK) {
            return IMU_ERR_COMM;
        }
        if (fifo_st.empty) {
            break;
        }
        FIFO_ReadWord(&dummy);
    } while (1);

    return IMU_OK;
}


uint16_t LSM6DSM_FIFO_GetAvailableSamples(void) {
    LSM6DSM_FifoStatus_t fifo_st;
    if (LSM6DSM_FIFO_GetStatus(&fifo_st) != IMU_OK) {
        return 0;
    }
    return fifo_st.sample_count;
}

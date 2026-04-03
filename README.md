# LSM6DSM IMU Library

This is a portable, hardware-abstracted driver library for the **ST LSM6DSM** 6-axis IMU (3-axis accelerometer + 3-axis gyroscope). The library has **no dependency on any specific microcontroller or HAL layer**. Platform-specific I/O (SPI/I2C read/write) is supplied by the user through function pointer callbacks, making the library drop-in compatible with any embedded target (STM32, ESP32, Arduino, bare-metal, RTOS, etc.).

A built-in **simulation mode** allows the entire pipeline—including filters and attitude estimation—to be tested on a PC without any real hardware, by injecting raw data through `LSM6DSM_InjectRawData()`.

---

### Project Structure

```
LSM6DSM_Hardware_Abstracted_Library/
├── inc/
│   ├── lsm6dsm_defs.h      # Register map, sensitivity constants, data types
│   ├── lsm6dsm_driver.h    # Core driver API (init, read, FIFO, callbacks)
│   ├── lsm6dsm_process.h   # Filter pipeline API
│   ├── filter.h            # All filter type definitions and API
│   └── ring_buffer.h       # Circular buffer for raw IMU samples
└── src/
    ├── lsm6dsm_driver.c    # Driver implementation
    ├── lsm6dsm_process.c   # Filter pipeline implementation
    ├── filter.c            # Filter implementations
    └── ring_buffer.c       # Ring buffer implementation
```

---

### Filtering

The library provides four independent, ready-to-use digital filter implementations in `filter.h` / `filter.c`. All filters operate sample-by-sample (no block processing required).

#### 1. 2nd-Order Bessel Low-Pass Filter (LPF)

Applied to **all three accelerometer axes** and to all three gyroscope axes after the Notch stage.

- Bilinear transform design with Bessel prototype coefficients (`1.6221`, `2.206`) for maximally-flat group delay (linear phase response).
- Configurable cutoff frequency — default **20 Hz**.
- API: `LPF_Init()`, `LPF_Update()`, `LPF_Reset()`

#### 2. 2nd-Order Notch (Band-Stop) Filter

Applied to **all three gyroscope axes** before the LPF to suppress power-line EMI.

- Pre-warped bilinear transform for accurate placement of the notch frequency.
- Default center frequency: **50 Hz** (adjustable to 60 Hz for other regions), bandwidth: **10 Hz**.
- API: `Notch_Init()`, `Notch_Update()`, `Notch_Reset()`

#### 3. Mahony Filter (Quaternion-Based Attitude Estimation)

Fuses accelerometer and gyroscope data to produce **roll, pitch, and yaw** angles.

- Quaternion-based complementary observer with PI correction (proportional + integral terms).
- Default gains: `Kp = 5.0`, `Ki = 0.001`.
- The integral term continuously corrects gyroscope bias drift at runtime.
- Outputs Euler angles in degrees via `Mahony_GetEuler()`.
- API: `Mahony_Init()`, `Mahony_Update()`, `Mahony_GetEuler()`, `Mahony_Reset()`

#### 4. Complementary Filter

A lightweight attitude filter for applications where the Mahony filter is too heavy.

- Blends gyroscope rate integration with accelerometer-based angle measurement.
- Configurable alpha (trust coefficient) and sample time.
- API: `CF_Init()`, `CF_Update()`, `CF_Reset()`

#### Default Filter Pipeline (in `lsm6dsm_process.c`)

```
Accelerometer  →  Bessel LPF  →  accel_x/y/z  ──┐
                                                   ├──► Mahony  →  roll / pitch / yaw
Gyroscope  →  Notch  →  Bessel LPF  →  gyro_x/y/z ┘
```

---

### How to Use

#### Step 1 — Implement Platform Callbacks

Provide read and write functions that match your hardware interface:

```c
// SPI or I2C write — must return 0 on success
int32_t MyWrite(void *handle, uint8_t reg, const uint8_t *data, uint16_t len) {
    // call your HAL write here (e.g., HAL_SPI_Transmit, HAL_I2C_Mem_Write)
    return 0;
}

// SPI or I2C read — must return 0 on success
int32_t MyRead(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {
    // call your HAL read here
    return 0;
}

// Optional: millisecond timestamp source (e.g., HAL_GetTick)
uint32_t MyGetTime(void *handle) {
    return HAL_GetTick();
}
```

#### Step 2 — Initialize the Driver

```c
#include "lsm6dsm_driver.h"
#include "lsm6dsm_process.h"

LSM6DSM_Config_t cfg = {
    .accel_scale = ACCEL_FS_4G,
    .gyro_scale  = GYRO_FS_500,
    .odr         = ODR_104HZ,        // 104 Hz output data rate
};

LSM6DSM_Status_t status = LSM6DSM_Init(MyWrite, MyRead, MyGetTime, NULL, &cfg);

if (status != IMU_OK) {
    // handle error
}

// Verify device identity
if (!LSM6DSM_CheckID()) {
    // wrong device or communication failure
}
```

#### Step 3 — Initialize the Filter Pipeline

```c
LSM6DSM_Process_t proc = {0};

// NULL config → uses default: LPF 20 Hz, Notch 50 Hz / 10 Hz, Mahony Kp=5, Ki=0.001
// NULL user_fn → uses the built-in filter pipeline
LSM6DSM_Process_Init(&proc,
                     LSM6DSM_GetSampleRate(),   // reads ODR from driver
                     NULL,                       // use default filter config
                     MyGetTime,                  // time source for bias calibration
                     NULL);                      // use built-in process callback
```

> **Note:** `LSM6DSM_Process_Init()` automatically performs a **200-sample gyroscope bias calibration** at startup. Keep the sensor stationary during initialization.

#### Step 4 — Read Data

**Raw (unscaled) data:**
```c
LSM6DSM_RawData_t raw;
LSM6DSM_ReadRaw(&raw);
// raw.accel_x/y/z  — int16_t LSB counts
// raw.gyro_x/y/z   — int16_t LSB counts
// raw.temperature  — int16_t raw ADC
```

**Scaled (physical units, no filtering):**
```c
LSM6DSM_Data_t scaled;
LSM6DSM_ReadScaled(&scaled);
// scaled.accel_x/y/z  — float [g]
// scaled.gyro_x/y/z   — float [dps]
// scaled.temp_c        — float [°C]
```

**Filtered + Attitude (full pipeline output):**
```c
LSM6DSM_FilteredData_t filtered;
LSM6DSM_ReadProcessed(&filtered);
// filtered.accel_x/y/z  — float [g]   (Bessel LPF applied)
// filtered.gyro_x/y/z   — float [dps] (Notch + Bessel LPF applied)
// filtered.roll_deg      — float [°]   (Mahony)
// filtered.pitch_deg     — float [°]   (Mahony)
// filtered.yaw_deg       — float [°]   (gyro integration only)
```

#### Custom Filter Callback

You can supply your own processing callback instead of the built-in pipeline:

```c
LSM6DSM_Status_t MyProcess(const LSM6DSM_Data_t *in,
                            LSM6DSM_FilteredData_t *out,
                            void *ctx) {
    // apply your own filters here
    out->accel_x = in->accel_x;
    // ...
    return IMU_OK;
}

LSM6DSM_Process_Init(&proc,
                     LSM6DSM_GetSampleRate(), 
                     NULL,                     
                     MyGetTime,               
                     MyProcess);                
```

#### FIFO Usage

```c
// Initialize FIFO with default settings (104 Hz, Continuous mode, watermark=200)
LSM6DSM_FIFO_Init(NULL);

// Or with custom settings:
LSM6DSM_FifoConfig_t fifo_cfg = {
    .odr       = REG_ODR_104HZ,
    .mode      = FIFO_MODE_CONT,
    .threshold = 300,
    .gyro_dec  = FIFO_DEC_1,
    .accel_dec = FIFO_DEC_1,
};
LSM6DSM_FIFO_Init(&fifo_cfg);

// Check FIFO status
LSM6DSM_FifoStatus_t fifo_status;
LSM6DSM_FIFO_GetStatus(&fifo_status);

// Stop FIFO
LSM6DSM_FIFO_Stop();
```

#### Simulation Mode (PC Testing)

Pass `NULL` for all platform callbacks. Inject sensor data manually:

```c
LSM6DSM_Init(NULL, NULL, NULL, NULL, &cfg);

LSM6DSM_RawData_t sim_data = {
    .accel_x = 0, .accel_y = 0, .accel_z = 16384,  // 1g on Z axis (2g scale)
    .gyro_x  = 0, .gyro_y  = 0, .gyro_z  = 0,
    .temperature = 0,
    .timestamp_ms = 10,
};
LSM6DSM_InjectRawData(&sim_data);
```

#### Shutdown

```c
LSM6DSM_DeInit();
```

---

### Status Codes

| Code | Meaning |
|------|---------|
| `IMU_OK` | Success |
| `IMU_ERR_COMM` | Communication error |
| `IMU_ERR_TIMEOUT` | Operation timed out |
| `IMU_ERR_INVALID_ID` | WHO_AM_I mismatch |
| `IMU_ERR_NOT_READY` | Data/device not ready |
| `IMU_ERR_BUFFER_FULL` | Ring buffer overflow |
| `IMU_ERR_BUFFER_EMPTY` | Ring buffer empty |
| `IMU_ERR_NULL_PTR` | Null pointer passed |
| `IMU_ERR_INVALID_PARAM` | Invalid parameter value |

---

## License

This project is open-source. Feel free to use, modify, and distribute it with attribution.

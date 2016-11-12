#pragma once

typedef struct gyroConfig_s {
    uint8_t gyroMovementCalibrationThreshold;   // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    uint8_t gyro_lpf;                           // gyro LPF setting - values are driver specific, in case of invalid number, a reasonable default ~30-40HZ is chosen.
    uint16_t soft_gyro_lpf_hz;                  // Software based gyro filter in hz
} gyroConfig_t;

PG_DECLARE(gyroConfig_t, gyroConfig);

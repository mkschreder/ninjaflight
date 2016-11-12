#pragma once

typedef struct barometerConfig_s {
    uint8_t baro_sample_count;              // size of baro filter array
    float baro_noise_lpf;                   // additional LPF to reduce baro noise
    float baro_cf_vel;                      // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity)
    float baro_cf_alt;                      // apply CF to use ACC for height estimation
} barometerConfig_t;

PG_DECLARE_PROFILE(barometerConfig_t, barometerConfig);

#pragma once

typedef struct gtuneConfig_s {
    uint8_t  gtune_lolimP[FD_INDEX_COUNT];  // [0..200] Lower limit of P during G tune
    uint8_t  gtune_hilimP[FD_INDEX_COUNT];  // [0..200] Higher limit of P during G tune. 0 Disables tuning for that axis.
    uint8_t  gtune_pwr;                     // [0..10] Strength of adjustment
    uint16_t gtune_settle_time;             // [200..1000] Settle time in ms
    uint8_t  gtune_average_cycles;          // [8..128] Number of looptime cycles used for gyro average calculation
} gtuneConfig_t;

PG_DECLARE_PROFILE(gtuneConfig_t, gtuneConfig);


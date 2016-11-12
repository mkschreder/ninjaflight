#pragma once

typedef enum {
    INPUT_FILTERING_DISABLED = 0,
    INPUT_FILTERING_ENABLED
} inputFilteringMode_e;

typedef struct pwmRxConfig_s {
    inputFilteringMode_e inputFilteringMode;  // Use hardware input filtering, e.g. for OrangeRX PPM/PWM receivers.
} pwmRxConfig_t;

PG_DECLARE(pwmRxConfig_t, pwmRxConfig);

#pragma once

typedef struct blackboxConfig_s {
    uint8_t rate_num;
    uint8_t rate_denom;
    uint8_t device;
} blackboxConfig_t;

PG_DECLARE(blackboxConfig_t, blackboxConfig);

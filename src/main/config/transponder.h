#pragma once

typedef struct transponderConfig_s {
    uint8_t data[6];
} transponderConfig_t;

PG_DECLARE(transponderConfig_t, transponderConfig);

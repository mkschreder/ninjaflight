#pragma once

typedef enum {
    GIMBAL_MODE_NORMAL = 0,
    GIMBAL_MODE_MIXTILT = 1,
	GIMBAL_MODE_MAX
} gimbalMode_e;

typedef struct gimbalConfig_s {
    uint8_t mode;
} gimbalConfig_t;

PG_DECLARE_PROFILE(gimbalConfig_t, gimbalConfig);

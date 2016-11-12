#pragma once

typedef struct hottTelemetryConfig_s {
    uint8_t hottAlarmSoundInterval;
} hottTelemetryConfig_t;

PG_DECLARE(hottTelemetryConfig_t, hottTelemetryConfig);

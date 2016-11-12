#pragma once

typedef struct telemetryConfig_s {
    uint8_t telemetry_switch;               // Use aux channel to change serial output & baudrate( MSP / Telemetry ). It disables automatic switching to Telemetry when armed.
    uint8_t telemetry_inversion;            // also shared with smartport inversion
} telemetryConfig_t;

PG_DECLARE(telemetryConfig_t, telemetryConfig);

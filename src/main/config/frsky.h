#pragma once

typedef enum {
    FRSKY_FORMAT_DMS = 0,
    FRSKY_FORMAT_NMEA
} frskyGpsCoordFormat_e;

typedef enum {
    FRSKY_UNIT_METRICS = 0,
    FRSKY_UNIT_IMPERIALS
} frskyUnit_e;

typedef struct frskyTelemetryConfig_s {
    float gpsNoFixLatitude;
    float gpsNoFixLongitude;
    frskyGpsCoordFormat_e frsky_coordinate_format;
    frskyUnit_e frsky_unit;
    uint8_t frsky_vfas_precision;
} frskyTelemetryConfig_t;

PG_DECLARE(frskyTelemetryConfig_t, frskyTelemetryConfig);

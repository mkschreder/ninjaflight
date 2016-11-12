#pragma once

#include "parameter_group.h"

typedef enum {
    ALIGN_DEFAULT = 0,                                      // driver-provided alignment
    CW0_DEG = 1,
    CW90_DEG = 2,
    CW180_DEG = 3,
    CW270_DEG = 4,
    CW0_DEG_FLIP = 5,
    CW90_DEG_FLIP = 6,
    CW180_DEG_FLIP = 7,
    CW270_DEG_FLIP = 8
} sensor_align_e;

typedef struct sensorAlignmentConfig_s {
    sensor_align_e gyro_align;              // gyro alignment
    sensor_align_e acc_align;               // acc alignment
    sensor_align_e mag_align;               // mag alignment
} sensorAlignmentConfig_t;

typedef struct sensorSelectionConfig_s {
    uint8_t acc_hardware;                   // Which acc hardware to use on boards with more than one device
    uint8_t mag_hardware;                   // Which mag hardware to use on boards with more than one device
    uint8_t baro_hardware;                  // Barometer hardware to use
} sensorSelectionConfig_t;

typedef struct int16_flightDynamicsTrims_s {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} flightDynamicsTrims_def_t;

typedef union {
    int16_t raw[3];
    flightDynamicsTrims_def_t values;
} flightDynamicsTrims_t;

typedef struct sensorTrims_s {
    flightDynamicsTrims_t accZero;
    flightDynamicsTrims_t magZero;
} sensorTrims_t;

PG_DECLARE(sensorSelectionConfig_t, sensorSelectionConfig);
PG_DECLARE(sensorAlignmentConfig_t, sensorAlignmentConfig);
PG_DECLARE(sensorTrims_t, sensorTrims);


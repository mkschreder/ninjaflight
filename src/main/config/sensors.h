/*
 * This file is part of Ninjaflight.
 *
 * Copyright: collective.
 * Cleanup: Martin Schröder <mkschreder.uk@gmail.com>
 * Original source from Cleanflight.
 *
 * Ninjaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Ninjaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Ninjaflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

typedef enum {
    SENSOR_GYRO = 1 << 0, // always present
    SENSOR_ACC = 1 << 1,
    SENSOR_BARO = 1 << 2,
    SENSOR_MAG = 1 << 3,
    SENSOR_SONAR = 1 << 4,
    SENSOR_GPS = 1 << 5,
    SENSOR_GPSMAG = 1 << 6,
} sensors_e;

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

struct sensor_alignment_config {
    sensor_align_e gyro_align;              // gyro alignment
    sensor_align_e acc_align;               // acc alignment
    sensor_align_e mag_align;               // mag alignment
} __attribute__((packed)) ;

struct sensor_selection_config {
    uint8_t acc_hardware;                   // Which acc hardware to use on boards with more than one device
    uint8_t mag_hardware;                   // Which mag hardware to use on boards with more than one device
    uint8_t baro_hardware;                  // Barometer hardware to use
} __attribute__((packed)) ;

typedef struct int16_flightDynamicsTrims_s {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} __attribute__((packed))  flightDynamicsTrims_def_t;

typedef union {
    int16_t raw[3];
    flightDynamicsTrims_def_t values;
} __attribute__((packed))  flightDynamicsTrims_t;

struct sensor_trims_config {
    flightDynamicsTrims_t accZero;
    flightDynamicsTrims_t magZero;
} __attribute__((packed)) ;

struct sensor_config {
	struct sensor_selection_config selection;
	struct sensor_alignment_config alignment;
	struct sensor_trims_config trims;
} __attribute__((packed)) ;



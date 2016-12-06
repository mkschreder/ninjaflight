/*
 * This file is part of Ninjaflight.
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

extern mag_t mag;

typedef enum {
    SENSOR_INDEX_GYRO = 0,
    SENSOR_INDEX_ACC,
    SENSOR_INDEX_BARO,
    SENSOR_INDEX_MAG
} sensorIndex_e;

#define MAX_SENSORS_TO_DETECT (SENSOR_INDEX_MAG + 1)

extern uint8_t detectedSensors[MAX_SENSORS_TO_DETECT];

// TODO: remove all of these private things when done refactoring
extern sensor_align_e gyroAlign;
extern sensor_align_e accAlign;
extern sensor_align_e magAlign;

bool sensorsAutodetect(const struct config *config);

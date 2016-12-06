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

#include "../config/barometer.h"

typedef enum {
    BARO_DEFAULT = 0,
    BARO_NONE = 1,
    BARO_BMP085 = 2,
    BARO_MS5611 = 3,
    BARO_BMP280 = 4
} baroSensor_e;

#define BARO_SAMPLE_COUNT_MAX   48
#define BARO_MAX BARO_BMP280

struct baro {
	int32_t BaroAlt;
	int32_t baroTemperature;             // Use temperature for telemetry

	const struct config *config;
};

void baro_init(struct baro *self, const struct config *config);
bool baro_is_calibrated(struct baro *self);
void baro_start_calibration(struct baro *self);
uint32_t baro_update(struct baro *self);
bool baro_is_ready(struct baro *self);
int32_t baro_calc_altitude(struct baro *self);

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

#include "drivers/accgyro.h"
#include "../common/filter.h"
#include "../common/maths.h"
#include "../config/gyro.h"
#include "../config/sensors.h"

struct ins_gyro {
	sensor_align_e align;

	int32_t gyroADC[XYZ_AXIS_COUNT];

	uint16_t calibratingG;
	int32_t gyroZero[XYZ_AXIS_COUNT];

	biquad_t gyroFilterState[3];
	bool use_filter;

	int32_t g[3];
	stdev_t var[3];

	const struct gyro_config *config;

	float gyro_scale;
};

void ins_gyro_init(struct ins_gyro *self, const struct gyro_config *config);
void ins_gyro_process_sample(struct ins_gyro *self, int32_t x, int32_t y, int32_t z);
void ins_gyro_calibrate(struct ins_gyro *self);
bool ins_gyro_is_calibrated(struct ins_gyro *self);

static inline void ins_gyro_set_scale(struct ins_gyro *self, float scale) { self->gyro_scale = scale; }

static inline int32_t ins_gyro_get_x(struct ins_gyro *self) { return self->gyroADC[X]; }
static inline int32_t ins_gyro_get_y(struct ins_gyro *self) { return self->gyroADC[Y]; }
static inline int32_t ins_gyro_get_z(struct ins_gyro *self) { return self->gyroADC[Z]; }

void ins_gyro_set_filter_hz(struct ins_gyro *self, uint16_t hz);

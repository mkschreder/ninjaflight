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

// TODO: remove dependency on compass 
#include "drivers/compass.h"
#include "../common/axis.h"
#include "../config/sensors.h"
#include "../config/compass.h"

struct ins_mag {
	int32_t magADC[XYZ_AXIS_COUNT];

	float magneticDeclination;

	float mag_scale[3];
	int16_t mag_max[3];
	int16_t mag_min[3];

	const struct mag_config *config;

	struct sensor_trims_config trims;

	int16_t calibratingM;
};

void ins_mag_init(struct ins_mag *self, const struct mag_config *config, const struct sensor_trims_config *trims);
void ins_mag_process_sample(struct ins_mag *self, int32_t x, int32_t y, int32_t z);

void ins_mag_save_trims(const struct ins_mag *self, struct config *config);

void ins_mag_start_calibration(struct ins_mag *self);
static inline bool ins_mag_is_calibrated(struct ins_mag *self) { return self->calibratingM == 0; }

static inline int32_t ins_mag_get_x(struct ins_mag *self) { return self->magADC[X]; }
static inline int32_t ins_mag_get_y(struct ins_mag *self) { return self->magADC[Y]; }
static inline int32_t ins_mag_get_z(struct ins_mag *self) { return self->magADC[Z]; }

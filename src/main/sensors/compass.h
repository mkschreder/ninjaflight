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
	int16_t magADCRaw[XYZ_AXIS_COUNT];
	int32_t magADC[XYZ_AXIS_COUNT];

	float magneticDeclination;

	uint32_t tCal;
    flightDynamicsTrims_t magZeroTempMin;
    flightDynamicsTrims_t magZeroTempMax;

	struct mag_config *config;
	struct sensor_trims_config *trims;

	bool calibrating;
};

void ins_mag_init(struct ins_mag *self, struct mag_config *config, struct sensor_trims_config *trims);
void ins_mag_process_sample(struct ins_mag *self, int32_t x, int32_t y, int32_t z);

static inline int32_t ins_mag_get_x(struct ins_mag *self) { return self->magADC[X]; }
static inline int32_t ins_mag_get_y(struct ins_mag *self) { return self->magADC[Y]; }
static inline int32_t ins_mag_get_z(struct ins_mag *self) { return self->magADC[Z]; }

//void recalculateMagneticDeclination(void);

/*
extern sensor_align_e magAlign;
extern mag_t mag;
extern float magneticDeclination;
*/

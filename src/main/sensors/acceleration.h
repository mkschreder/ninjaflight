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

#include "../drivers/accgyro.h"
#include "../common/axis.h"
#include "../config/accelerometer.h"
#include "../config/sensors.h"

struct ins_acc {
	int32_t accADC[XYZ_AXIS_COUNT];
	uint16_t calibratingA;
	int32_t a[3];
	int32_t b[3];
	//int16_t accZero_saved[3];
	//rollAndPitchTrims_t angleTrim_saved;
	struct accelerometer_config *config;
	struct sensor_trims_config *trims;
	int16_t acc_1G;
};
//extern int32_t accADC[XYZ_AXIS_COUNT];

void ins_acc_init(struct ins_acc *self, struct sensor_trims_config *trims, struct accelerometer_config *config);
void ins_acc_process_sample(struct ins_acc *self, int32_t x, int32_t y, int32_t z);
void ins_acc_calibrate(struct ins_acc *self);
bool ins_acc_is_calibrated(struct ins_acc *self);

void ins_acc_set_scale(struct ins_acc *self, int16_t acc_1G);

static inline int32_t ins_acc_get_x(struct ins_acc *self) { return self->accADC[X]; }
static inline int32_t ins_acc_get_y(struct ins_acc *self) { return self->accADC[Y]; }
static inline int32_t ins_acc_get_z(struct ins_acc *self) { return self->accADC[Z]; }


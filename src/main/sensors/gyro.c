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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <platform.h>

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/config.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/config_reset.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gyro_sync.h"

#include "sensors/sensors.h"

#include "io/beeper.h"
#include "io/statusindicator.h"

#include "sensors/boardalignment.h"

#include "sensors/gyro.h"

#define CALIBRATING_GYRO_CYCLES			 1000


void ins_gyro_init(struct ins_gyro *self, struct gyro_config *config, float gyro_scale){
	memset(self, 0, sizeof(struct ins_gyro));
	self->config = config;
	self->gyro_scale = gyro_scale;
	self->calibratingG = CALIBRATING_GYRO_CYCLES;

	if (config->soft_gyro_lpf_hz) {
		// Initialisation needs to happen once sampling rate is known
		for (int axis = 0; axis < 3; axis++) {
			BiQuadNewLpf(config->soft_gyro_lpf_hz, &self->gyroFilterState[axis], 1000);
		}
		self->use_filter = true;
	}
}

static void _add_calibration_samples(struct ins_gyro *self, int32_t raw[3]){
	for (int axis = 0; axis < 3; axis++) {
		// Reset g[axis] at start of calibration
		if (self->calibratingG == CALIBRATING_GYRO_CYCLES) {
			self->g[axis] = 0;
			devClear(&self->var[axis]);
		}

		// Sum up CALIBRATING_GYRO_CYCLES readings
		self->g[axis] += self->gyroADC[axis];
		devPush(&self->var[axis], self->gyroADC[axis]);

		// Reset global variables to prevent other code from using un-calibrated data
		self->gyroZero[axis] = 0;

		if (self->calibratingG == 1) {
			float dev = devStandardDeviation(&self->var[axis]);
			// check deviation and startover in case the model was moved
			if (self->config->move_threshold && dev > self->config->move_threshold) {
				self->calibratingG = CALIBRATING_GYRO_CYCLES;
				return;
			}
			self->gyroZero[axis] = (self->g[axis] + (CALIBRATING_GYRO_CYCLES / 2)) / CALIBRATING_GYRO_CYCLES;
		}
	}

	self->calibratingG--;
}

void ins_gyro_process_sample(struct ins_gyro *self, int32_t x, int32_t y, int32_t z){
	int32_t raw[3] = { x, y, z };

	if (self->use_filter) {
		for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
			raw[axis] = lrintf(applyBiQuadFilter((float)raw[axis], &self->gyroFilterState[axis]));
		}
	}

	if (self->calibratingG > 0) {
		_add_calibration_samples(self, raw);
	} else {
		// only update values if we have been calibrated
		for (int axis = 0; axis < 3; axis++) {
			self->gyroADC[axis] = raw[axis] - self->gyroZero[axis];
		}
	}
}

void ins_gyro_calibrate(struct ins_gyro *self){
	for(int c = 0; c < 3; c++)
		self->gyroADC[c] = 0;
	self->calibratingG = CALIBRATING_GYRO_CYCLES;
}

bool ins_gyro_is_calibrated(struct ins_gyro *self){
	return self->calibratingG == 0;
}


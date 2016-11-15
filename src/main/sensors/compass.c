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

#include <platform.h>

#include "build_config.h"

#include "common/axis.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/sensor.h"
#include "drivers/compass.h"
#include "drivers/compass_hmc5883l.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"

#include "sensors/boardalignment.h"
#include "config/runtime_config.h"
#include "config/config.h"

#include "sensors/sensors.h"
#include "sensors/compass.h"

#ifdef NAZE
#include "hardware_revision.h"
#endif

void ins_mag_init(struct ins_mag *self, struct mag_config *config, struct sensor_trims_config *trims){
	memset(self, 0, sizeof(struct ins_mag));
	self->config = config;
	self->trims = trims;

	if (config->mag_declination > 0) {
		// calculate magnetic declination
		int16_t deg = config->mag_declination / 100;
		int16_t min = config->mag_declination % 100;

		self->magneticDeclination = (deg + ((float)min * (1.0f / 60.0f))) * 10; // heading is in 0.1deg units
	} else {
		self->magneticDeclination = 0.0f; // TODO investigate if this is actually needed if there is no mag sensor or if the value stored in the config should be used.
	}
}

void ins_mag_process_sample(struct ins_mag *self, int32_t x, int32_t y, int32_t z){
	flightDynamicsTrims_t *magZero = &self->trims->magZero;
	// TODO: fix this
	int32_t currentTime = 0;
	
	self->magADC[X] = x;
	self->magADC[Y] = y;
	self->magADC[Z] = z;

	if (self->calibrating) {
		self->tCal = currentTime;
		for (int axis = 0; axis < 3; axis++) {
			magZero->raw[axis] = 0;
			self->magZeroTempMin.raw[axis] = self->magADC[axis];
			self->magZeroTempMax.raw[axis] = self->magADC[axis];
		}
		self->calibrating = 0;
	}

	if (self->tCal != 0) {
		if ((currentTime - self->tCal) < 30000000) {	// 30s: you have 30s to turn the multi in all directions
			for (int axis = 0; axis < 3; axis++) {
				if (self->magADC[axis] < self->magZeroTempMin.raw[axis])
					self->magZeroTempMin.raw[axis] = self->magADC[axis];
				if (self->magADC[axis] > self->magZeroTempMax.raw[axis])
					self->magZeroTempMax.raw[axis] = self->magADC[axis];
			}
		} else {
			self->tCal = 0;
			for (int axis = 0; axis < 3; axis++) {
				magZero->raw[axis] = (self->magZeroTempMin.raw[axis] + self->magZeroTempMax.raw[axis]) / 2; // Calculate offsets
			}

			//saveConfigAndNotify();
		}
	} else {
		self->magADC[X] -= magZero->raw[X];
		self->magADC[Y] -= magZero->raw[Y];
		self->magADC[Z] -= magZero->raw[Z];
	}
}

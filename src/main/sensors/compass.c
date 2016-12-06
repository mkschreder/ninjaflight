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

#include "sensors/boardalignment.h"
#include "config/config.h"

#include "sensors/compass.h"

#ifdef NAZE
#include "hardware_revision.h"
#endif

#define MAG_CALIBRATION_SAMPLES 240

void ins_mag_init(struct ins_mag *self, const struct mag_config *config, const struct sensor_trims_config *trims){
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

void ins_mag_start_calibration(struct ins_mag *self){
	for(int c = 0; c < 3; c++) {
		self->mag_min[c] = 0x7fff;
		self->mag_max[c] = -0x7fff;
		self->mag_scale[c] = 0;
	}
	self->calibratingM = MAG_CALIBRATION_SAMPLES;
}

void ins_mag_process_sample(struct ins_mag *self, int32_t x, int32_t y, int32_t z){
	const flightDynamicsTrims_t *magZero = &self->trims->magZero;

	int32_t raw[3] = { x, y, z };

	if(self->calibratingM > 0){
		// find min and max
		for(int c = 0; c < 3; c++){
			if(raw[c] < self->mag_min[c]) self->mag_min[c] = raw[c];
			if(raw[c] > self->mag_max[c]) self->mag_max[c] = raw[c];
		}
		if(self->calibratingM == 1){
			// Get hard iron correction
			// TODO: save mag trims
			/*
			magZero->raw[X] = ((int32_t)self->mag_max[0] + self->mag_min[0])/2;  // get average x mag bias in counts
			magZero->raw[Y] = ((int32_t)self->mag_max[1] + self->mag_min[1])/2;  // get average y mag bias in counts
			magZero->raw[Z] = ((int32_t)self->mag_max[2] + self->mag_min[2])/2;  // get average z mag bias in counts
			*/
			// Get soft iron correction estimate
			self->mag_scale[0]  = ((int32_t)self->mag_max[0] - self->mag_min[0])/2;  // get average x axis max chord length in counts
			self->mag_scale[1]  = ((int32_t)self->mag_max[1] - self->mag_min[1])/2;  // get average y axis max chord length in counts
			self->mag_scale[2]  = ((int32_t)self->mag_max[2] - self->mag_min[2])/2;  // get average z axis max chord length in counts

			float avg_rad = (self->mag_scale[0] + self->mag_scale[1] + self->mag_scale[2]) / 3.0f;

			self->mag_scale[0] = avg_rad/((float)self->mag_scale[0]);
			self->mag_scale[1] = avg_rad/((float)self->mag_scale[1]);
			self->mag_scale[2] = avg_rad/((float)self->mag_scale[2]);
		}
		self->calibratingM--;
	} else {
		self->magADC[X] = raw[X] - magZero->raw[X];
		self->magADC[Y] = raw[Y] - magZero->raw[Y];
		self->magADC[Z] = raw[Z] - magZero->raw[Z];
	}
}

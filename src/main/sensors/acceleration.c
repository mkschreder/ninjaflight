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

#include "config/config.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/battery.h"
#include "sensors/boardalignment.h"

#include "config/runtime_config.h"
#include "config/config_reset.h"
#include "config/feature.h"

#include "acceleration.h"

#define CALIBRATING_ACC_CYCLES              400
/*
extern uint16_t InflightcalibratingA;
extern bool AccInflightCalibrationArmed;
extern bool AccInflightCalibrationMeasurementDone;
extern bool AccInflightCalibrationSavetoEEProm;
extern bool AccInflightCalibrationActive;

static flightDynamicsTrims_t *accelerationTrims;
*/

void ins_acc_init(struct ins_acc *self, struct sensor_trims_config *trims, struct accelerometer_config *config){
	memset(self, 0, sizeof(struct ins_acc));
	self->calibratingA = 0; // do not calibrate by default since this can lead to weird effects
	self->config = config;
	self->trims = trims;

	ins_acc_set_scale(self, 512);
}

void ins_acc_set_scale(struct ins_acc *self, int16_t acc_1G){
	self->acc_1G = acc_1G;

	// reset acceleration
	self->accADC[0] = 0;
	self->accADC[1] = 0;
	self->accADC[2] = self->acc_1G;
}

#if 0
static void performInflightAccelerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims){
	uint8_t axis;
	
	// Saving old zeropoints before measurement
	if (InflightcalibratingA == 50) {
		self->accZero_saved[X] = accelerationTrims->raw[X];
		self->accZero_saved[Y] = accelerationTrims->raw[Y];
		self->accZero_saved[Z] = accelerationTrims->raw[Z];
		angleTrim_saved.values.roll = rollAndPitchTrims->values.roll;
		angleTrim_saved.values.pitch = rollAndPitchTrims->values.pitch;
	}
	if (InflightcalibratingA > 0) {
		for (int axis = 0; axis < 3; axis++) {
			// Reset a[axis] at start of calibration
			if (InflightcalibratingA == 50)
				self->b[axis] = 0;
			// Sum up 50 readings
			self->b[axis] += self->accADC[axis];
			// Clear global variables for next reading
			self->accADC[axis] = 0;
			accelerationTrims->raw[axis] = 0;
		}
		// all values are measured
		if (InflightcalibratingA == 1) {
			AccInflightCalibrationActive = false;
			AccInflightCalibrationMeasurementDone = true;
			beeper(BEEPER_ACC_CALIBRATION); // indicate end of calibration
			// recover saved values to maintain current flight behaviour until new values are transferred
			accelerationTrims->raw[X] = accZero_saved[X];
			accelerationTrims->raw[Y] = accZero_saved[Y];
			accelerationTrims->raw[Z] = accZero_saved[Z];
			rollAndPitchTrims->values.roll = angleTrim_saved.values.roll;
			rollAndPitchTrims->values.pitch = angleTrim_saved.values.pitch;
		}
		InflightcalibratingA--;
	}
	// Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
	if (AccInflightCalibrationSavetoEEProm) {	  // the aircraft is landed, disarmed and the combo has been done again
		AccInflightCalibrationSavetoEEProm = false;
		accelerationTrims->raw[X] = b[X] / 50;
		accelerationTrims->raw[Y] = b[Y] / 50;
		accelerationTrims->raw[Z] = b[Z] / 50 - acc.acc_1G;	// for nunchuck 200=1G

		resetRollAndPitchTrims(rollAndPitchTrims);

		//saveConfigAndNotify();
	}
}
#endif

void ins_acc_process_sample(struct ins_acc *self, int32_t x, int32_t y, int32_t z){
	int32_t raw[3] = { x, y, z };
	if(self->calibratingA > 0){
		for (int axis = 0; axis < 3; axis++) {
			// Sum up CALIBRATING_ACC_CYCLES readings
			self->a[axis] += raw[axis];
		}

		if (self->calibratingA == 1) {
			// Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
			self->trims->accZero.raw[X] = self->a[X] / CALIBRATING_ACC_CYCLES;
			self->trims->accZero.raw[Y] = self->a[Y] / CALIBRATING_ACC_CYCLES;
			self->trims->accZero.raw[Z] = self->a[Z] / CALIBRATING_ACC_CYCLES - self->acc_1G;

			//trims->values.roll = 0;
			//trims->values.pitch = 0;
		}
		self->calibratingA--;

		self->accADC[X] = 0;
		self->accADC[Y] = 0;
		self->accADC[Z] = self->acc_1G; // NOTE: this is actually wrong because z should be down and gravity force is always up then, but cleanflight had it wrong so for now we have to comply so other modules continue to work!
	} else {
		self->accADC[X] = x - self->trims->accZero.raw[X];
		self->accADC[Y] = y - self->trims->accZero.raw[Y];
		self->accADC[Z] = z - self->trims->accZero.raw[Z];
	}
/*
	if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
		performInflightAccelerationCalibration(rollAndPitchTrims);
	}
*/
}

void ins_acc_calibrate(struct ins_acc *self){
	for(int axis = 0; axis < 3; axis++)
		self->a[axis] = 0;
	self->calibratingA = CALIBRATING_ACC_CYCLES;
}

bool ins_acc_is_calibrated(struct ins_acc *self){
	return self->calibratingA == 0;
}


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
#include "build_config.h"

#include "common/maths.h"

#include "drivers/barometer.h"
#include "drivers/system.h"

#include "config/config.h"

#include "sensors/barometer.h"

#include "system_calls.h"

#define CALIBRATING_BARO_CYCLES			 200 // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles

baro_t baro;						// barometer access functions

bool isBaroCalibrationComplete(struct baro *self)
{
	return self->calibratingB == 0;
}

void baroSetCalibrationCycles(struct baro *self, uint16_t calibrationCyclesRequired)
{
	self->calibratingB = calibrationCyclesRequired;
}


static int32_t applyBarometerMedianFilter(struct baro *self, int32_t newPressureReading)
{
	int nextSampleIndex;
	
	nextSampleIndex = (self->currentFilterSampleIndex + 1);
	if (nextSampleIndex == PRESSURE_SAMPLES_MEDIAN) {
		nextSampleIndex = 0;
		self->medianFilterReady = true;
	}

	self->barometerFilterSamples[self->currentFilterSampleIndex] = newPressureReading;
	self->currentFilterSampleIndex = nextSampleIndex;
	
	if (self->medianFilterReady)
		return quickMedianFilter3(self->barometerFilterSamples);
	else
		return newPressureReading;
}

#define PRESSURE_SAMPLE_COUNT (config_get_profile(self->config)->baro.baro_sample_count - 1)

static uint32_t recalculateBarometerTotal(struct baro *self, uint8_t baroSampleCount, uint32_t pressureTotal, int32_t newPressureReading)
{
	int nextSampleIndex;

	// store current pressure in barometerSamples
	nextSampleIndex = (self->currentSampleIndex + 1);
	if (nextSampleIndex == baroSampleCount) {
		nextSampleIndex = 0;
		self->baroReady = true;
	}
	self->barometerSamples[self->currentSampleIndex] = applyBarometerMedianFilter(self, newPressureReading);

	// recalculate pressure total
	// Note, the pressure total is made up of baroSampleCount - 1 samples - See PRESSURE_SAMPLE_COUNT
	pressureTotal += self->barometerSamples[self->currentSampleIndex];
	pressureTotal -= self->barometerSamples[nextSampleIndex];

	self->currentSampleIndex = nextSampleIndex;

	return pressureTotal;
}

void baro_init(struct baro *self, const struct config *config){
	memset(self, 0, sizeof(*self));
	self->config = config;
	self->state = BAROMETER_NEEDS_SAMPLES;
}

bool isBaroReady(struct baro *self) {
	return self->baroReady;
}
/*
static PT_THREAD(_fsm_baro(struct baro *self)){
	uint32_t pressure = 0;
	PT_BEGIN(&self->state);
	while(true){
		PT_WAIT_UNTIL(sys_read_pressure(self->system, &pressure) == 0);
	}
	PT_END();
}
*/

void baro_process_pressure(struct baro *self, uint32_t pressure){
	self->baroPressureSum = recalculateBarometerTotal(self, config_get_profile(self->config)->baro.baro_sample_count, self->baroPressureSum, pressure);
}

void baro_update(struct baro *self){
	int32_t BaroAlt_tmp;
	
	const struct barometer_config *conf = &config_get_profile(self->config)->baro;
	// calculates height from ground via baro readings
	// see: https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.cpp#L140
	BaroAlt_tmp = lrintf((1.0f - powf((float)(self->baroPressureSum / PRESSURE_SAMPLE_COUNT) / 101325.0f, 0.190295f)) * 4433000.0f); // in cm
	BaroAlt_tmp -= self->baroGroundAltitude;
	self->BaroAlt = lrintf((float)self->BaroAlt * conf->baro_noise_lpf + (float)BaroAlt_tmp * (1.0f - conf->baro_noise_lpf)); // additional LPF to reduce baro noise
/*
	switch (self->state) {
		default:
		case BAROMETER_NEEDS_SAMPLES:
			baro.get_ut();
			baro.start_up();
			self->state = BAROMETER_NEEDS_CALCULATION;
			return baro.up_delay;
		break;

		case BAROMETER_NEEDS_CALCULATION:
			baro.get_up();
			baro.start_ut();
			baro.calculate(&self->baroPressure, &self->baroTemperature);
			self->baroPressureSum = recalculateBarometerTotal(self, config_get_profile(self->config)->baro.baro_sample_count, self->baroPressureSum, self->baroPressure);
			self->state = BAROMETER_NEEDS_SAMPLES;
			return baro.ut_delay;
		break;
	}
	*/
}

uint32_t baro_get_altitude(struct baro *self){
	return self->BaroAlt;
}

void performBaroCalibrationCycle(struct baro *self){
	self->baroGroundPressure -= self->baroGroundPressure / 8;
	self->baroGroundPressure += self->baroPressureSum / PRESSURE_SAMPLE_COUNT;
	self->baroGroundAltitude = (1.0f - powf((self->baroGroundPressure / 8) / 101325.0f, 0.190295f)) * 4433000.0f;

	self->calibratingB--;
}


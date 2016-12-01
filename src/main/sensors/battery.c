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

#include "stdbool.h"
#include "stdint.h"
#include "string.h"

#include <platform.h>

#include "common/maths.h"
#include "common/filter.h"

#include "drivers/adc.h"
#include "drivers/system.h"

#include "config/config.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "config/runtime_config.h"
#include "config/feature.h"

#include "sensors/battery.h"

#define VBATT_PRESENT_THRESHOLD_MV	10
#define VBATT_LPF_FREQ  1.0f

void battery_init(struct battery *self, struct battery_config *config){
	memset(self, 0, sizeof(struct battery));
	self->config = config;
	self->batteryCellCount = 3;
	self->batteryState = BATTERY_NOT_PRESENT;
	self->batteryCellCount = 1;
	self->batteryWarningVoltage = 0;
	self->batteryCriticalVoltage = 0;

	BiQuadNewLpf(VBATT_LPF_FREQ, &self->vbatFilterState, 50000);
}

uint16_t battery_get_voltage(struct battery *self){
	return self->vbat;
}

uint8_t battery_get_cell_count(struct battery *self){
	return self->batteryCellCount;
}

int32_t battery_get_current(struct battery *self){
	return self->amperage;
}

int32_t battery_get_spent_capacity(struct battery *self){
	return self->mAhDrawn;
}

uint16_t battery_get_cell_voltage(struct battery *self){
	return ((uint32_t)self->vbat * 100 + self->batteryCellCount) / (self->batteryCellCount * 2);
}

// TODO: make static after we refactor unit tests
uint16_t _battery_adc_to_voltage(struct battery *self, uint16_t src);
uint16_t _battery_adc_to_voltage(struct battery *self, uint16_t src){
	// calculate battery voltage based on ADC reading
	// result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 0xFFF = 12bit adc, 110 = 11:1 voltage divider (10k:1k) * 10 for 0.1V
	return ((((uint32_t)src * self->config->vbatscale * 33 + (0xFFF * 5)) / (0xFFF * self->config->vbatresdivval)) / self->config->vbatresdivmultiplier);
}

static void _battery_update_voltage(struct battery *self){
	uint16_t vbatSample;
	// store the battery voltage with some other recent battery voltage readings
	vbatSample = self->vbatLatestADC = adcGetChannel(ADC_BATTERY);
	vbatSample = applyBiQuadFilter(vbatSample, &self->vbatFilterState);
	self->vbat = _battery_adc_to_voltage(self, vbatSample);
}

#define VBATTERY_STABLE_DELAY 40
/* Batt Hysteresis of +/-100mV */
#define VBATT_HYSTERESIS 1

void battery_update(struct battery *self){
	_battery_update_voltage(self);

	/* battery has just been connected*/
	if (self->batteryState == BATTERY_NOT_PRESENT && self->vbat > VBATT_PRESENT_THRESHOLD_MV){
		/* Actual battery state is calculated below, this is really BATTERY_PRESENT */
		self->batteryState = BATTERY_OK;
		/* wait for VBatt to stabilise then we can calc number of cells
		(using the filtered value takes a long time to ramp up) 
		We only do this on the ground so don't care if we do block, not
		worse than original code anyway*/
		usleep(VBATTERY_STABLE_DELAY * 1000);
		_battery_update_voltage(self);

		unsigned cells = (_battery_adc_to_voltage(self, self->vbatLatestADC) / self->config->vbatmaxcellvoltage) + 1;
		if (cells > 8) {
			// something is wrong, we expect 8 cells maximum (and autodetection will be problematic at 6+ cells)
			cells = 8;
		}
		self->batteryCellCount = cells;
		self->batteryWarningVoltage = self->batteryCellCount * self->config->vbatwarningcellvoltage;
		self->batteryCriticalVoltage = self->batteryCellCount * self->config->vbatmincellvoltage;
	}
	/* battery has been disconnected - can take a while for filter cap to disharge so we use a threshold of VBATT_PRESENT_THRESHOLD_MV */
	else if (self->batteryState != BATTERY_NOT_PRESENT && self->vbat <= VBATT_PRESENT_THRESHOLD_MV)
	{
		self->batteryState = BATTERY_NOT_PRESENT;
		self->batteryCellCount = 0;
		self->batteryWarningVoltage = 0;
		self->batteryCriticalVoltage = 0;
	}	
}

battery_state_t battery_get_state(struct battery *self){
	return self->batteryState;
}

const char * battery_get_state_str(struct battery *self){
	static const char * const batteryStateStrings[] = {"OK", "WARNING", "CRITICAL", "NOT PRESENT"};
	return batteryStateStrings[self->batteryState];
}

#define ADCVREF 3300   // in mV
static int32_t _battery_current_to_centiamps(struct battery *self, uint16_t src)
{
	int32_t millivolts;

	millivolts = ((uint32_t)src * ADCVREF) / 4096;
	millivolts -= self->config->currentMeterOffset;

	return (millivolts * 1000) / (int32_t)self->config->currentMeterScale; // current in 0.01A steps
}

void battery_update_current_meter(struct battery *self, int32_t lastUpdateAt){
	static int32_t amperageRaw = 0;
	static int64_t mAhdrawnRaw = 0;
	switch(self->config->currentMeterType) {
		case CURRENT_SENSOR_ADC:
			amperageRaw -= amperageRaw / 8;
			amperageRaw += (self->amperageLatestADC = adcGetChannel(ADC_CURRENT));
			self->amperage = _battery_current_to_centiamps(self, amperageRaw / 8);
			break;
		case CURRENT_SENSOR_VIRTUAL:
			self->amperage = (int32_t)self->config->currentMeterOffset;
			break;
		case CURRENT_SENSOR_NONE:
			self->amperage = 0;
			break;
	}

	// TODO: current meter notification
	#if 0
	int32_t throttleOffset = (int32_t)rcCommand[THROTTLE] - 1000;
	int32_t throttleFactor = 0;
	if (self->config->currentMeterType == CURRENT_SENSOR_VIRTUAL && ARMING_FLAG(ARMED)) {
		if (throttleStatus == THROTTLE_LOW && feature(FEATURE_MOTOR_STOP))
			throttleOffset = 0;
		throttleFactor = throttleOffset + (throttleOffset * throttleOffset / 50);
		self->amperage += throttleFactor * (int32_t)self->config->currentMeterScale  / 1000;
	}
	#endif

	mAhdrawnRaw += (MAX(0, self->amperage) * lastUpdateAt) / 1000;
	self->mAhDrawn = mAhdrawnRaw / (3600 * 100);
}

uint8_t battery_get_remaining_percent(struct battery *self)
{
	return (((uint32_t)self->vbat - (self->config->vbatmincellvoltage * self->batteryCellCount)) * 100) / ((self->config->vbatmaxcellvoltage - self->config->vbatmincellvoltage) * self->batteryCellCount);
}

uint8_t battery_get_remaining_capacity(struct battery *self){
	uint16_t batteryCapacity = self->config->batteryCapacity;

	return constrain((batteryCapacity - constrain(self->mAhDrawn, 0, 0xFFFF)) * 100.0f / batteryCapacity , 0, 100);
}

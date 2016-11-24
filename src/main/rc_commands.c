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
#include <stddef.h>

#include <platform.h>

#include "build_config.h"

#include "common/maths.h"
#include "common/axis.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "io/rc_curves.h"
#include "io/rc_controls.h"

#include "rc_commands.h"
#include "rx/rx.h"


#define RC_MID 1500
#define RC_MIN 1000
#define RC_MAX 2000

static void _update_curves(struct rc_command *self){
	int32_t rp_expo = (self->config)?self->config->rcExpo8:0;
	int32_t yaw_expo = (self->config)?self->config->rcYawExpo8:0;
	int32_t thr_mid = (self->config)?self->config->thrMid8:0;
	int32_t thr_expo = (self->config)?self->config->thrExpo8:0;

	for (int i = 0; i < PITCH_LOOKUP_LENGTH; i++)
		self->lookup_roll_pitch[i] = (2500 + rp_expo * (i * i - 25)) * i * rp_expo / 2500;

	for (int i = 0; i < YAW_LOOKUP_LENGTH; i++)
		self->lookup_yaw[i] = (2500 + yaw_expo * (i * i - 25)) * i / 25;

	for (int i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
		int16_t tmp = 10 * i - thr_mid;
		uint8_t y = 1;
		if (tmp > 0)
			y = 100 - thr_mid;
		if (tmp < 0)
			y = thr_mid;
		self->lookup_throttle[i] = 10 * thr_mid + tmp * (100 - thr_expo + (int32_t) thr_expo * (tmp * tmp) / (y * y)) / 10;
		// TODO: this should be done in the mixer and not here.
		//self->lookup_throttle[i] = motorAndServoConfig()->minthrottle + (int32_t) (motorAndServoConfig()->maxthrottle - motorAndServoConfig()->minthrottle) * lookupThrottleRC[i] / 1000; // [MINTHROTTLE;MAXTHROTTLE]
	}
}

#if 0
static void _filter_rc_commands(struct ninja *self, float dt){
	// TODO: remove the statics
	static int16_t lastCommand[4] = { 0, 0, 0, 0 };
	static int16_t deltaRC[4] = { 0, 0, 0, 0 };
	static int16_t factor, rcInterpolationFactor;
	uint16_t rxRefreshRate = rc_get_refresh_rate()

	rcInterpolationFactor = rxRefreshRate / (dt * 1000000UL);

	if (self->isRXDataNew) {
		for (int channel=0; channel < 4; channel++) {
			deltaRC[channel] = rcCommand[channel] -  (lastCommand[channel] - deltaRC[channel] * factor / rcInterpolationFactor);
			lastCommand[channel] = rcCommand[channel];
		}

		self->isRXDataNew = false;
		factor = rcInterpolationFactor - 1;
	} else {
		factor--;
	}

	// Interpolate steps of rcCommand
	if (factor > 0) {
		for (int channel=0; channel < 4; channel++) {
			rcCommand[channel] = lastCommand[channel] - deltaRC[channel] * factor/rcInterpolationFactor;
		 }
	} else {
		factor = 0;
	}
}
#endif 

void rc_command_init(struct rc_command *self, struct rx *rx){
	memset(self, 0, sizeof(struct rc_command));
	self->rx = rx;
}

//! Looks up expo value. Expects table to be at least 6 elements
static int16_t _lookup_expo(int16_t *table, int16_t input){
	// convert rc input into range of 0;500 as an absolute deflection value
	int16_t rc = MIN(ABS(input - RC_MID), 500);
	// limit to value of 0-5
	int16_t idx = constrain(rc / 100, 0, 5);
	// this is the "rest" or rc % 100
	int16_t rc_mod = rc - idx * 100;
	// get table lookup value which is going to be from 0 to 500 and subject to the expo based on current value (0;500) as x axis.
	int16_t lookup = table[idx] + rc_mod * (table[idx + 1] - table[idx]) / 100;
	if(input < RC_MID) return -lookup;
	return lookup;
}

void rc_command_update(struct rc_command *self){
	int16_t roll = rx_get_channel(self->rx, ROLL);
	int16_t pitch = rx_get_channel(self->rx, PITCH);
	int16_t yaw = rx_get_channel(self->rx, YAW);
	int16_t throttle = rx_get_channel(self->rx, THROTTLE);

	// ensure that we can work with default settings even without config
	int32_t tpa_breakpoint = (self->config)?constrain(self->config->tpa_breakpoint, RC_MIN, RC_MAX):RC_MIN;
	int32_t dyn_thr_pid = (self->config)?constrain(self->config->dynThrPID, 0, 100):0;

	// calculate new tpa value in range [0;100]
	if (throttle < tpa_breakpoint) {
		self->tpa = 100;
	} else {
		int16_t t = throttle - tpa_breakpoint;
		// tpa goes from 100 down to (100 - dyn_thr_pid) since dyn_thr_pid is maximum pid reduction in percent
		self->tpa = 100 - dyn_thr_pid * t / (RC_MAX - tpa_breakpoint);
	}

	if(self->config){
		self->roll = _lookup_expo(self->lookup_roll_pitch, roll);
		self->pitch = _lookup_expo(self->lookup_roll_pitch, pitch);
		self->yaw = _lookup_expo(self->lookup_yaw, yaw);
		self->throttle = _lookup_expo(self->lookup_throttle, yaw);
	} else {
		self->roll = roll - 1500;
		self->pitch = pitch - 1500;
		self->yaw = yaw - 1500;
		self->throttle = throttle - 1500;
	}

	/*
	if (rxConfig()->rcSmoothing) {
		_filter_rc_commands(self, dt);
	}
	*/
}

//! Get current stick value in range [-500;500], stick is only ROLL, PITCH, YAw
int16_t rc_command_axis(struct rc_command *self, uint8_t axis){
	switch(axis){
		case ROLL: return self->roll;
		case PITCH: return self->pitch;
		case YAW: return self->yaw;
		case THROTTLE: return self->throttle;
		default: return 0;
	}
}

//! Get current angle command in decidegrees, range [-450;450]
int16_t rc_command_angle(struct rc_command *self, uint8_t stick){
	switch(stick){
		case 0: return self->roll * 450 / 500;
		case 1: return self->pitch * 450 / 500;
		case 2: return self->yaw * 450 / 500;
		default: return 0;
	}
}

//! Get rate command in degrees per second (with rate config applied), range [-3625;3625]
int16_t rc_command_rate(struct rc_command *self, uint8_t stick){
	switch(stick){
		case 0: return (16 + self->config->rates[0]) * self->roll / 16;
		case 1: return (16 + self->config->rates[1]) * self->pitch / 16;
		case 2: return (16 + self->config->rates[2]) * self->yaw / 16;
		default: return 0;
	}
}

//! Updates internal variables to use new rates and also recalculates the lookup tables if the supplied profile is different than the one that is currently being used.
void rc_command_set_rate_config(struct rc_command *self, struct rate_config *rates){
	struct rate_config *cur = self->config;
	self->config = rates;
	if(cur != rates)
		_update_curves(self);
}


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

#include "io/rc_controls.h"
#include "flight/anglerate.h"

#include "sensors/barometer.h"
#include "../config/altitudehold.h"

struct instruments;
struct althold {
	int32_t setVelocity;
	uint8_t velocityControl;
	int32_t errorVelocityI;
	int32_t altHoldThrottleAdjustment;
	int32_t AltHold;
	int32_t vario;                      // variometer in cm/s

	int16_t initialRawThrottleHold;
	int16_t initialThrottleHold;
	int32_t EstAlt;                // in cm

    uint8_t isAltHoldChanged;
	struct instruments *ins;
};

void althold_init(struct althold *self, struct instruments *ins);

void althold_calc_altitude(struct althold *self, uint32_t currentTime);

void althold_apply(struct althold *self);
void althold_update(struct althold *self);
void althold_update_sonar(struct althold *self);

int32_t althold_get_est_altitude(struct althold *self);

/*
 * This file is part of Ninjaflight.
 *
 * Copyright: collective.
 * Cleanup: Martin Schröder <mkschreder.uk@gmail.com>
 * Original source from Cleanflight.
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

#include "rc_controls.h"

typedef struct adjustmentRange_s {
	// when aux channel is in range...
	uint8_t auxChannelIndex;
	channelRange_t range;

	// ..then apply the adjustment function to the auxSwitchChannel ...
	uint8_t adjustmentFunction;
	uint8_t auxSwitchChannelIndex;

	// ... via slot
	uint8_t adjustmentIndex;
} adjustmentRange_t;

#define MAX_ADJUSTMENT_RANGE_COUNT 12 // enough for 2 * 6pos switches.

struct rc_adjustment_profile {
    adjustmentRange_t adjustmentRanges[MAX_ADJUSTMENT_RANGE_COUNT];
};



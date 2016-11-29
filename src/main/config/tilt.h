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

#include "parameter_group.h"

#define MIXER_TILT_COMPENSATE_THRUST (1 << 0)
#define MIXER_TILT_COMPENSATE_TILT (1 << 1)
#define MIXER_TILT_COMPENSATE_BODY (1 << 2)

//! mixer tilt mode type
typedef enum {
	MIXER_TILT_MODE_STATIC,
	MIXER_TILT_MODE_DYNAMIC
} mixer_tilt_mode_t;


struct tilt_config {
	uint8_t mode;
	uint8_t compensation_flags;
	uint8_t control_channel;
	int8_t servo_angle_min;
	int8_t servo_angle_max;
};

PG_DECLARE(struct tilt_config, tiltConfig);

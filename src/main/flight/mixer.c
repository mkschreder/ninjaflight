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

/**
 * \file mixer.c
 *
 * Mixer implementation
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <platform.h>
#include "build_config.h"

#include "mixer.h"

enum mixer_flags {
	//! mixer has to be armed in order to make motor calculations. Motor values are set to either mincommand or minthrottle depending on whether motor stop is enabled
	MIXER_FLAG_ARMED =		(1 << 3),
};

/// @cond
#ifndef USE_QUAD_MIXER_ONLY
#define USE_QUAD_MIXER_ONLY 0
#endif

#define COUNT_SERVO_RULES(rules) (sizeof(rules) / sizeof(struct servo_mixer))
/// @endcond

/* Updated mixer rules below. Note that even though these look larger, they in
 * fact take up less memory than the old ones due to better memory efficiency
 * and elimination of zero rules */

/* QuadX
4CW   2CCW
   \ /
	X
   / \
3CCW  1CW
*/
static const struct mixer_rule_def mixerQuadX[] = {
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },

	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_PITCH,	-1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_THROTTLE,	 1000 },

	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_THROTTLE,	 1000 },

	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_PITCH,	-1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_THROTTLE,	 1000 },
#if USE_TILT == 1
	{ MIXER_OUTPUT_S1, MIXER_INPUT_G2_GIMBAL_PITCH,  1000 },
	{ MIXER_OUTPUT_S2, MIXER_INPUT_G2_GIMBAL_PITCH,	-1000 },
#endif
};

/* QuadP
	4CW
	 |
3CCW-+-2CCW
	 |
	1CW
*/
static const struct mixer_rule_def mixerQuadP[] = {
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },

	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_THROTTLE,	 1000 },

	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_THROTTLE,	 1000 },

	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_PITCH,	-1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_THROTTLE,	 1000 },
};

/* Vtail4
4CCW-----2CW
	 ||
	 ||
3CW\ || /1CCW
	\||/
*/
static const struct mixer_rule_def mixerVtail4[] = {
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_ROLL,		-580 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_PITCH,	 580 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },

	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_ROLL,		-460 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_PITCH,	-390 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_YAW,		-500 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_THROTTLE,	 1000 },

	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_ROLL,		 580 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_PITCH,	 580 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_THROTTLE,	 1000 },

	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_ROLL,		 460 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_PITCH,	-390 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_YAW,		 500 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_THROTTLE,	 1000 },
};

/* Atail4
 4CW----2CCW
	 ||
	 ||
	 /\
3CCW/  \1CW
*/
static const struct mixer_rule_def mixerAtail4[] = {
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_ROLL,		-580 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_PITCH,	 580 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },

	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_ROLL,		-460 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_PITCH,	-390 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_YAW,		 500 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_THROTTLE,	 1000 },

	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_ROLL,		 580 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_PITCH,	 580 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_THROTTLE,	 1000 },

	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_ROLL,		 460 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_PITCH,	-390 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_YAW,		-500 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_THROTTLE,	 1000 },
};

/* Y4
 4CW----2CCW
	 ||
	 ||
	1CW
	3CCW
*/
static const struct mixer_rule_def mixerY4[] = {
	// REAR_TOP CW
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// FRONT_R CCW
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_PITCH,	-1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// REAR_BOTTOM CCW
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// FRONT_L CW
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_PITCH,	-1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_THROTTLE,	 1000 },
};

static const struct mixer_rule_def mixerY6[] = {
	// REAR
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_PITCH,	 1333 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// RIGHT
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_PITCH,	-666 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// LEFT
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_PITCH,	-666 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// UNDER_REAR
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_PITCH,	 1333 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// UNDER_RIGHT
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_PITCH,	-666 },
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// UNDER_LEFT
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_PITCH,	-666 },
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_THROTTLE,	 1000 },
};

static const struct mixer_rule_def mixerHex6H[] = {
	// REAR_R
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// FRONT_R
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_PITCH,	-1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// REAR_L
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// FRONT_L
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_PITCH,	-1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// RIGHT
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// LEFT
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_THROTTLE,	 1000 },
};

static const struct mixer_rule_def mixerHex6P[] = {
	// REAR_R
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_ROLL,		-866 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_PITCH,	 500 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// FRONT_R
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_ROLL,		-866 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_PITCH,	-500 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// REAR_L
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_ROLL,		 866 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_PITCH,	 500 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// FRONT_L
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_ROLL,		 866 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_PITCH,	-500 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// FRONT
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_PITCH,	-1000 },
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// REAR
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_THROTTLE,	 1000 },
};

static const struct mixer_rule_def mixerHex6X[] = {
	// REAR_R
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_ROLL,		-500 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_PITCH,	 866 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// FRONT_R
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_ROLL,		-500 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_PITCH,	-866 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// REAR_L
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_ROLL,		 500 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_PITCH,	 866 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// FRONT_L
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_ROLL,		 500 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_PITCH,	-866 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// RIGHT
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// LEFT
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_THROTTLE,	 1000 },
};

static const struct mixer_rule_def mixerOctoX8[] = {
	// REAR_R
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// FRONT_R
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_PITCH,	-1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// REAR_L
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// FRONT_L
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_PITCH,	-1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// UNDER_REAR_R
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// UNDER_FRONT_R
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_PITCH,	-1000 },
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// UNDER_REAR_L
	{ MIXER_OUTPUT_M7, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M7, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_M7, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M7, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// UNDER_FRONT_L
	{ MIXER_OUTPUT_M8, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M8, MIXER_INPUT_G0_PITCH,	-1000 },
	{ MIXER_OUTPUT_M8, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M8, MIXER_INPUT_G0_THROTTLE,	 1000 },
};

static const struct mixer_rule_def mixerOctoFlatP[] = {
	// FRONT_L
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_ROLL,		 707 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_PITCH,	-707 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// FRONT_R
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_ROLL,		-707 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_PITCH,	-707 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// REAR_R
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_ROLL,		-707 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_PITCH,	 707 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// REAR_L
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_ROLL,		 707 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_PITCH,	 707 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// FRONT
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_PITCH,	-1000 },
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// RIGHT
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// REAR
	{ MIXER_OUTPUT_M7, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_M7, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M7, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// LEFT
	{ MIXER_OUTPUT_M8, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M8, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M8, MIXER_INPUT_G0_THROTTLE,	 1000 },
};

static const struct mixer_rule_def mixerOctoFlatX[] = {
	// MIDFRONT_L
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_PITCH,	-414 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// FRONT_R
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_ROLL,		-414 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_PITCH,	-1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// MIDREAR_R
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_PITCH,	 414 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// REAR_L
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_ROLL,		 414 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M4, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// FRONT_L
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_ROLL,		 414 },
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_PITCH,	-1000 },
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M5, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// MIDFRONT_R
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_PITCH,	-414 },
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M6, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// REAR_R
	{ MIXER_OUTPUT_M7, MIXER_INPUT_G0_ROLL,		-414 },
	{ MIXER_OUTPUT_M7, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_M7, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M7, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// MIDREAR_L
	{ MIXER_OUTPUT_M8, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M8, MIXER_INPUT_G0_PITCH,	 414 },
	{ MIXER_OUTPUT_M8, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M8, MIXER_INPUT_G0_THROTTLE,	 1000 },
};

static const struct mixer_rule_def mixerAirplane[] = {
	// MOTOR1
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// FLAPPERON_1
	{ MIXER_OUTPUT_S1, MIXER_INPUT_G0_ROLL,		 1000 },
	// FLAPPERON_2
	{ MIXER_OUTPUT_S2, MIXER_INPUT_G0_ROLL,		 1000 },
	// RUDDER
	{ MIXER_OUTPUT_S3, MIXER_INPUT_G0_YAW,		 1000 },
	// ELEVATOR
	{ MIXER_OUTPUT_S4, MIXER_INPUT_G0_PITCH,	 1000 },
	// THROTTLE (for gas planes)
	{ MIXER_OUTPUT_S5, MIXER_INPUT_G0_THROTTLE,	 1000 },
};

static const struct mixer_rule_def mixerFlyingWing[] = {
	// MOTOR1
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// FLAPPERON_1
	{ MIXER_OUTPUT_S1, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_S1, MIXER_INPUT_G0_PITCH,	 1000 },
	// FLAPPERON_2
	{ MIXER_OUTPUT_S2, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_S2, MIXER_INPUT_G0_PITCH,	-1000 },
	// THROTTLE (for gas planes)
	{ MIXER_OUTPUT_S3, MIXER_INPUT_G0_THROTTLE,	 1000 },
};

static const struct mixer_rule_def mixerSingleCopter[] = {
	// MOTOR1
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// SINGLECOPTER_1
	{ MIXER_OUTPUT_S1, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_S1, MIXER_INPUT_G0_YAW,		 1000 },
	// SINGLECOPTER_2
	{ MIXER_OUTPUT_S2, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_S2, MIXER_INPUT_G0_YAW,		 1000 },
	// SINGLECOPTER_3
	{ MIXER_OUTPUT_S3, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_S3, MIXER_INPUT_G0_YAW,		 1000 },
	// SINGLECOPTER_4
	{ MIXER_OUTPUT_S4, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_S4, MIXER_INPUT_G0_YAW,		 1000 },
};

static const struct mixer_rule_def mixerDualcopter[] = {
	// LEFT
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_YAW,		-1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// RIGHT
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_YAW,		 1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// PITCH
	{ MIXER_OUTPUT_S1, MIXER_INPUT_G0_PITCH,	 1000 },
	// ROLL
	{ MIXER_OUTPUT_S2, MIXER_INPUT_G0_ROLL,		 1000 },
};

static const struct mixer_rule_def mixerBicopter[] = {
	// LEFT
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// RIGHT
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// LEFT
	{ MIXER_OUTPUT_S1, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_S1, MIXER_INPUT_G0_YAW,		 1000 },
	// RIGHT
	{ MIXER_OUTPUT_S2, MIXER_INPUT_G0_PITCH,	 1000 },
	{ MIXER_OUTPUT_S2, MIXER_INPUT_G0_YAW,		 1000 },
};

static const struct mixer_rule_def mixerTricopter[] = {
	// REAR
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_PITCH,	 1333 },
	{ MIXER_OUTPUT_M1, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// RIGHT
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_ROLL,		-1000 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_PITCH,	-666 },
	{ MIXER_OUTPUT_M2, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// LEFT
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_ROLL,		 1000 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_PITCH,	-666 },
	{ MIXER_OUTPUT_M3, MIXER_INPUT_G0_THROTTLE,	 1000 },
	// YAW
	{ MIXER_OUTPUT_S1, MIXER_INPUT_G0_YAW,		 1000 },
};

static const struct mixer_rule_def mixerGimbal[] = {
	{ MIXER_OUTPUT_S1, MIXER_INPUT_G2_GIMBAL_ROLL, 1000 },
	{ MIXER_OUTPUT_S2, MIXER_INPUT_G2_GIMBAL_PITCH,1000 },
};

static void _update_motor_and_servo_count(struct mixer *self){
	self->motorCount = 0;
	self->servoCount = 0;
	uint8_t motor[MIXER_OUTPUT_COUNT] = {0};
	uint8_t servo[MIXER_OUTPUT_COUNT] = {0};
	for(int c = 0; c < self->ruleCount; c++){
		struct mixer_rule_def *rule = &self->active_rules[c];
		if(rule->output < MIXER_OUTPUT_SERVOS) motor[rule->output] = true;
		else servo[rule->output] = true;
	}
	for(int c = 0; c < MIXER_OUTPUT_COUNT; c++){
		if(motor[c]) self->motorCount++;
		if(servo[c]) self->servoCount++;
	}
}

void mixer_load_preset(struct mixer *self, mixer_mode_t preset){
	int rule_count = 0;
	const struct mixer_rule_def *rules = NULL;
	// setting mixer like this makes sure that we save flash space
	/// @cond
	#define GET_PRESET(def) if(USE_QUAD_MIXER_ONLY == 0){ rules = def; rule_count = sizeof(def) / sizeof(def[0]); }
	#define GET_PRESET_QUADX(def) { rules = def; rule_count = sizeof(def) / sizeof(def[0]); }
	/// @endcond
	switch(preset){
		case MIXER_TRI:			GET_PRESET(mixerTricopter); break;
		case MIXER_QUADP:		GET_PRESET(mixerQuadP); break;
		case MIXER_QUADX:		GET_PRESET_QUADX(mixerQuadX); break;
		case MIXER_BICOPTER:	GET_PRESET(mixerBicopter); break;
		case MIXER_GIMBAL:		GET_PRESET(mixerGimbal); break;
		case MIXER_Y6:			GET_PRESET(mixerY6); break;
		case MIXER_HEX6:		GET_PRESET(mixerHex6P); break;
		case MIXER_FLYING_WING:	GET_PRESET(mixerFlyingWing); break;
		case MIXER_Y4:			GET_PRESET(mixerY4); break;
		case MIXER_HEX6X:		GET_PRESET(mixerHex6X); break;
		case MIXER_OCTOX8:		GET_PRESET(mixerOctoX8); break;
		case MIXER_OCTOFLATP:	GET_PRESET(mixerOctoFlatP); break;
		case MIXER_OCTOFLATX:	GET_PRESET(mixerOctoFlatX); break;
		case MIXER_AIRPLANE:	GET_PRESET(mixerAirplane); break;
		case MIXER_HELI_120_CCPM: break;
		case MIXER_HELI_90_DEG:	break;
		case MIXER_VTAIL4:		GET_PRESET(mixerVtail4); break;
		case MIXER_HEX6H:		GET_PRESET(mixerHex6H); break;
		case MIXER_PPM_TO_SERVO:	break;
		case MIXER_DUALCOPTER:	GET_PRESET(mixerDualcopter); break;
		case MIXER_SINGLECOPTER:GET_PRESET(mixerSingleCopter); break;
		case MIXER_ATAIL4:		GET_PRESET(mixerAtail4); break;
		case MIXER_QUADX_TILT1:	break;
		case MIXER_QUADX_TILT2:	break;
		case MIXER_CUSTOM:		break;
		case MIXER_CUSTOM_AIRPLANE: break;
		case MIXER_CUSTOM_TRI:	break;
		case MIXER_MODE_COUNT: break;
		default: break;
	};

	self->ruleCount = constrain(rule_count, 0, MIXER_MAX_RULES);
	memcpy(self->active_rules, rules, sizeof(struct mixer_rule_def) * self->ruleCount);

	_update_motor_and_servo_count(self);
}

/**
 * Initializes an empty mixer objects clearing memory first.
 */
void mixer_init(struct mixer *self,
	struct mixer_config *mixer_config,
	struct motor_3d_config *motor_3d_config,
	motorAndServoConfig_t *motor_servo_config,
	rxConfig_t *rx_config,
	rcControlsConfig_t *rc_controls_config,
	struct servo_config *servo_config,
	struct motor_mixer *initialCustomMixers, uint8_t count){
	(void)initialCustomMixers;
	(void)count;
	memset(self, 0, sizeof(struct mixer));

	self->mixer_config = mixer_config;
	self->motor_3d_config = motor_3d_config;
	self->motor_servo_config = motor_servo_config;
	self->rx_config = rx_config;
	self->rc_controls_config = rc_controls_config;
	self->servo_config = servo_config;

	// safety measure
	self->input[MIXER_INPUT_G0_THROTTLE] = -500;
	for(int c = 0; c < 8; c++){
		self->input[MIXER_INPUT_G4_M1 + c] = -500;
	}

	// set default throttle range
	mixer_set_throttle_range(self, 1500,
		self->motor_servo_config->minthrottle,
		self->motor_servo_config->maxthrottle);

	// load the configured mixer profile
	mixer_load_preset(self, mixer_config->mixerMode);
}

/**
 * Saves current motor mixer motor settings into the motor_mixer struct which
 * is exposed through the config
 *
 * @param output: where to write the motor_mixer rules in cleanflight format.
 *
 * @return number of motor_mixer rules written into output
 */
int mixer_save_motor_mixer(struct mixer *self, struct motor_mixer *output){
	for(int c = 0; c < MAX_SUPPORTED_MOTORS; c++){
		output[c].throttle = 0;
	}
	for(int c = 0; c < self->ruleCount; c++){
		struct mixer_rule_def *rule = &self->active_rules[c];
		if(rule->output > MIXER_OUTPUT_M8) continue;
		int i = rule->output - MIXER_OUTPUT_M1;
		if(i > MAX_SUPPORTED_MOTORS) continue;
		switch(rule->input){
			case MIXER_INPUT_G0_ROLL: output[i].roll = (float)rule->scale / 1000.0f; break;
			case MIXER_INPUT_G0_PITCH: output[i].pitch = (float)rule->scale / 1000.0f; break;
			case MIXER_INPUT_G0_YAW: output[i].yaw = (float)rule->scale / 1000.0f; break;
			case MIXER_INPUT_G0_THROTTLE: output[i].throttle = (float)rule->scale / 1000.0f; break;
			default:break;
		}
	}

	// count motors
	int ret = 0;
	for(int c = 0; c < MAX_SUPPORTED_MOTORS && output[c].throttle > 0; c++){
		ret++;
	}
	return ret;
}

/**
 * Loads motor mixer from the config into internal ruleset format
 */
void mixer_load_motor_mixer(struct mixer *self, const struct motor_mixer *motors){
	for(int c = 0; c < MAX_SUPPORTED_MOTORS; c++){
		const struct motor_mixer *rule = &motors[c];
		if(fabsf(rule->throttle) < 1e-6f) break;
		char found[4] = {0, 0, 0, 0};
		// note that this is very inefficient search, but we only do this rather rarely so it's ok
		for(int j = 0; j < self->ruleCount; j++){
			struct mixer_rule_def *ar = &self->active_rules[j];
			if(ar->input == MIXER_INPUT_G0_ROLL && ar->output == (MIXER_OUTPUT_MOTORS + c)){
				ar->scale = rule->roll * 1000.0f; found[0] = 1;
			}
			if(ar->input == MIXER_INPUT_G0_PITCH && ar->output == (MIXER_OUTPUT_MOTORS + c)){
				ar->scale = rule->pitch * 1000.0f; found[1] = 1;
			}
			if(ar->input == MIXER_INPUT_G0_YAW && ar->output == (MIXER_OUTPUT_MOTORS + c)){
				ar->scale = rule->yaw * 1000.0f; found[2] = 1;
			}
			if(ar->input == MIXER_INPUT_G0_THROTTLE && ar->output == (MIXER_OUTPUT_MOTORS + c)){
				ar->scale = rule->throttle * 1000.0f; found[3] = 1;
			}
		}
		if(!found[0] && fabsf(rule->roll) > 1e-6f)
			self->active_rules[self->ruleCount++] = (struct mixer_rule_def){ .input = MIXER_INPUT_G0_ROLL, .output = MIXER_OUTPUT_MOTORS + c, .scale = rule->roll * 1000.0f };
		if(!found[1] && fabsf(rule->pitch) > 1e-6f)
			self->active_rules[self->ruleCount++] = (struct mixer_rule_def){ .input = MIXER_INPUT_G0_PITCH, .output = MIXER_OUTPUT_MOTORS + c, .scale = rule->pitch * 1000.0f };
		if(!found[2] && fabsf(rule->yaw) > 1e-6f)
			self->active_rules[self->ruleCount++] = (struct mixer_rule_def){ .input = MIXER_INPUT_G0_YAW, .output = MIXER_OUTPUT_MOTORS + c, .scale = rule->yaw * 1000.0f };
		if(!found[3] && fabsf(rule->throttle) > 1e-6f)
			self->active_rules[self->ruleCount++] = (struct mixer_rule_def){ .input = MIXER_INPUT_G0_THROTTLE, .output = MIXER_OUTPUT_MOTORS + c, .scale = rule->throttle * 1000.0f };
	}
	_update_motor_and_servo_count(self);
}

/**
 * Loads servo mixer from mixer config
 */
void mixer_load_servo_mixer(struct mixer *self, const struct servo_mixer *servos){
	for(int c = 0; c < MAX_SUPPORTED_SERVOS; c++){
		const struct servo_mixer *rule = &servos[c];
		if(rule->rate == 0) break;
		bool found = false;
		// note that this is very inefficient search, but we only do this rather rarely so it's ok
		for(int j = 0; j < self->ruleCount; j++){
			if(self->active_rules[j].input == rule->inputSource && self->active_rules[j].output == rule->targetChannel){
				self->active_rules[j].scale = rule->rate * 10.0f;
				found = true;
				break;
			}
		}
		if(!found)
			self->active_rules[self->ruleCount++] = (struct mixer_rule_def){ .input = rule->inputSource, .output = rule->targetChannel, .scale = rule->rate * 10.0f };
	}
	_update_motor_and_servo_count(self);
}
/**
 * Exports servo settings from the internal rules representation into config servo_mixer
 */
int mixer_save_servo_mixer(struct mixer *self, struct servo_mixer *output){
	int ret = 0;
	for(int c = 0; c < MAX_SUPPORTED_SERVOS; c++){
		output[c].rate = 0;
	}
	for(int c = 0; c < self->ruleCount; c++){
		struct mixer_rule_def *rule = &self->active_rules[c];
		if(rule->output < MIXER_OUTPUT_S1 || rule->output > MIXER_OUTPUT_S8) continue;
		int i = rule->output - MIXER_OUTPUT_S1;
		if(i >= MAX_SUPPORTED_SERVOS) continue;
		output[i].inputSource = rule->input;
		output[i].targetChannel = rule->output;
		output[i].rate = (float)rule->scale / 10.0f;
		output[i].speed = 0;
		output[i].min = 0;
		output[i].max = 100;
		ret ++;
	}
	return ret;
}

void mixer_clear_rules(struct mixer *self){
	self->ruleCount = 0;
	memset(self->active_rules, 0, sizeof(self->active_rules));
}

/**
 * Inputs a command into the mixer. Valid range is [-500 to 500];
 *
 * @param channel the channel to set
 * @param value the value of the channel. Zero centered and between -500 to 500
 */
void mixer_input_command(struct mixer *self, mixer_input_t channel, int16_t value){
	// input is constrained between -500 to 500 for all channels in order to support negative values.
	self->input[channel] = constrain(value, -500, 500);
}

/**
 * Sets throttle range for scaling motor values. All outputs will be centered
 * at mid throttle and will be clipped to min/max range (this works because all
 * inputs are -500 to 500).
 *
 * @param mid the middle throttle value on which outputs will be centered.
 * @param min minimum possible output value
 * @param max maximum possible output value
 */
void mixer_set_throttle_range(struct mixer *self, int16_t mid, int16_t min, int16_t max){
	if(max < min) max = min;
	self->midthrottle = constrain(mid, min, max);
	self->minthrottle = min;
	self->maxthrottle = max;
}

static void _scale_motors(struct mixer *self, int16_t *output, uint16_t count){
	// find the minimum and maximum value
	int16_t minmotor = 0, maxmotor = 0;
	for(uint16_t c = 0; c < count; c++){
		if(output[c] < minmotor) minmotor = output[c];
		if(output[c] > maxmotor) maxmotor = output[c];
	}
	int16_t motorrange = maxmotor - minmotor;
	int16_t throttlerange = self->maxthrottle - self->minthrottle;
	if(motorrange > throttlerange && motorrange > 0){
		self->motorLimitReached = true;
		float scale = (float)throttlerange / motorrange;
		for(uint16_t c = 0; c < count; c++){
			output[c] = lrintf(output[c] * scale);
		}
	} else if((self->midthrottle + maxmotor) > self->maxthrottle) {
		self->motorLimitReached = true;
		// if we went over the top limit we move the throttle down
		int16_t offset = (self->midthrottle + maxmotor) - self->maxthrottle;
		for(uint16_t c = 0; c < count; c++){
			output[c] -= offset;
		}
	} else if((self->midthrottle + minmotor) < self->minthrottle){
		self->motorLimitReached = true;
		// if we went below the minimum throttle then we move the throttle up
		int16_t offset = self->minthrottle - (self->midthrottle + minmotor);
		for(uint16_t c = 0; c < count; c++){
			output[c] += offset;
		}
	} else {
		self->motorLimitReached = false;
	}
}

/**
 * Updates the outputs based on mixing rules from the inputs. Expects inputs to
 * be set using mixer_input_* command.
 *
 * If mixer is in disarmed state then it will forward group 4 inputs (motor
 * passthrough) to the outputs. This feature can be used to test motors when
 * mixer is not mixing (without changing mixing mode).
 */
void mixer_update(struct mixer *self){
	// if we are disarmed then we write preset disarmed values (this is necessary so we can test motors from configurator)
	if(!(self->flags & MIXER_FLAG_ARMED)){
		for(int c = 0; c < MIXER_MAX_MOTORS; c++){
			self->output[MIXER_OUTPUT_MOTORS + c] = self->midthrottle + self->input[MIXER_INPUT_GROUP_MOTOR_PASSTHROUGH + c];
		}
		goto finish;
	}

	// we will copy this into mixer output when we are done
	int16_t output[MIXER_OUTPUT_COUNT];
	memset(output, 0, sizeof(output));

	// mix all outputs according to rules. This will put range centered around zero into all output channels.
	for(int c = 0; c < self->ruleCount; c++){
		struct mixer_rule_def *rule = &self->active_rules[c];
		if(rule->input > MIXER_INPUT_COUNT || rule->output > MIXER_OUTPUT_COUNT) continue;
		output[rule->output] += (((int32_t)self->input[rule->input] * rule->scale) / 1000);
	}

	// since multiple motors can only mean differential thrust, we need to make sure that we fit the full motor range into the min/max throttle range.
	_scale_motors(self, output + MIXER_OUTPUT_MOTORS, MIXER_OUTPUT_SERVOS - MIXER_OUTPUT_MOTORS);

	// we center the outputs on midrc always regardless of midthrottle because
	for(int c = MIXER_OUTPUT_MOTORS; c < MIXER_MAX_MOTORS; c++){
		output[c] = constrain(self->midthrottle + output[c], self->minthrottle, self->maxthrottle);
	}

	// limit servos according to config
	for (int i = 0; i < MIXER_MAX_SERVOS; i++) {
		struct servo_config *conf = &self->servo_config[i];
		uint16_t servo_width = conf->max - conf->min;
		// TODO: the 0 and 100 were supposed to be part of servo mixer rule but never seemed to be used so replaced by constants for now.
		int16_t min = 0 * servo_width / 100 - servo_width / 2;
		int16_t max = 100 * servo_width / 100 - servo_width / 2;
		output[MIXER_OUTPUT_SERVOS + i] = conf->middle + constrain((output[MIXER_OUTPUT_SERVOS + i] * (int32_t)conf->rate) / 100L, min, max);
	}

	memcpy(self->output, output, sizeof(self->output));

finish:
	// forward rc channels to servos that are not controller by the mixer
	for(int i = self->servoCount, chan = 0; i < MIXER_MAX_SERVOS && chan < MIXER_INPUT_G3_RC_AUX3; i++, chan++){
		self->output[MIXER_OUTPUT_SERVOS + i] = self->midthrottle + self->input[MIXER_INPUT_G3_RC_AUX1+chan];
	}
}

//! arms/disarms the mixer (when disarmed, motor outputs will be set to disarmed pwm values. These are reset to either midrc when in 3d mode or to mincommand when not in 3d mode)
void mixer_enable_armed(struct mixer *self, bool on){
	if(on) self->flags |= MIXER_FLAG_ARMED;
	else self->flags &= ~MIXER_FLAG_ARMED;
}

static uint16_t _get_constrained_output(struct mixer *self, uint8_t id){
	int16_t val = self->output[id];
	return constrain(val, self->motor_servo_config->mincommand, 2000);
}

//! returns computed motor value for specified motor or 0 if motor is out of range
uint16_t mixer_get_motor_value(struct mixer *self, uint8_t id){
	if(id >= MIXER_MAX_MOTORS) return self->motor_servo_config->mincommand;
	return _get_constrained_output(self, MIXER_OUTPUT_MOTORS + id);
}

//! returns constrained servo output.
uint16_t mixer_get_servo_value(struct mixer *self, uint8_t id){
	if(id >= MIXER_MAX_SERVOS) return self->motor_servo_config->mincommand;
	return _get_constrained_output(self, MIXER_OUTPUT_SERVOS + id);
}

//! returns true if mixer has determined that motor limit has been reached
bool mixer_motor_limit_reached(struct mixer *self){
	return self->motorLimitReached;
}

//! returns the number of motors that are currently being controlled by mixer profile
uint8_t mixer_get_motor_count(struct mixer *self){
	return self->motorCount;
}

//! returns number of servos that are used by the mixer.
uint8_t mixer_get_servo_count(struct mixer *self){
	return self->servoCount;
}

/**
* @file mixer.c
*
* NinjaMixer
* ==========
*
* @brief the rewritten universal aircraft mixer
*
* @author Rewrite: Martin Schröder <mkschreder.uk@gmail.com>
* @author Original: Cleanflight
*
* The primary job of a mixer is to translate inputs from the flight controller
* into linear or angular movement of the aircraft body. The mixer does this by
* taking into account frame and thrust configuration and generating actuator
* values for required control surfaces.
*
* Inputs to the mixer
* ------------------
*
* Mixer inputs are divided into 4 groups:
* - MIXER_INPUT_GROUP_FC: group 0, flight controller stabilization inputs
* - MIXER_INPUT_GROUP_1: group 1, reserved
* - MIXER_INPUT_GROUP_GIMBAL: group 2, gimbal controls (also used by tilt drone)
* - MIXER_INPUT_GROUP_RC: group 3, rc controls
* - MIXER_INPUT_GROUP_MOTOR_PASSTHROUGH: group 4, motor passthrough
*
* Each input group contains 8 inputs and can be referenced by the mixer rules. 
*
* All inputs must be in range -500 to +500.
*
* Outputs of the mixer
* --------------------
*
* Mixer outputs are divided into 2 groups:
* - Motors
* - Servos
*
* Motor outpus are constrained by minthrottle and maxthrottle and centered at
* midthrottle value.
*
* Mixer will automatically forward servo outputs from RC_AUX1 channel and
* onwards into servo slots that come after the servos that are used by the
* mixer.
*
* Servo outputs are scaled and limited according to the ninjaflight servo
* configuration.
*
* Mixer rules
* -----------
*
* All mixer rules are additive to the given output channel. Each rule defines a
* scale operation for a particular input to the mixer.
*/



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
 * @module Flight Library
 *
 * The main responsibility of this class is to process raw rc commands and
 * translate them into values that are to be passed as commands to the flight
 * controller. This process involves applying expo and ensuring valid limits on
 * the output values.
 */

#pragma once

#include "config/rate_profile.h"

//! compile time configuration of the pitch/roll lookup table
#define PITCH_LOOKUP_LENGTH 7
//! compile time configuration of the yaw lookup table
#define YAW_LOOKUP_LENGTH 7
//! compile time configuration of the throttle lookup table
#define THROTTLE_LOOKUP_LENGTH 12

//! RC commands state. Inputs are raw commands in range [1000-2000]. Outputs are RC commands in range [-500;500] after applying expo and other parameters.
struct rc_command {
	int16_t roll, pitch, yaw, throttle;

	//! Throttle PID Addjustment. TODO: this really does not belong in this class.
	int16_t tpa;

	//! current rate config
	struct rate_config *config;

	int16_t lookup_roll_pitch[PITCH_LOOKUP_LENGTH];		//!< lookup table for expo & RC rate PITCH+ROLL
	int16_t lookup_yaw[YAW_LOOKUP_LENGTH];				//!< lookup table for expo & RC rate YAW
	int16_t lookup_throttle[THROTTLE_LOOKUP_LENGTH];	//!< lookup table for expo & mid THROTTLE
};

//! Initializes the curves with default linear ranges.
void rc_command_init(struct rc_command *self);

//! Updates internal variables to use new rates and also recalculates the lookup tables if the supplied profile is different than the one that is currently being used.
void rc_command_set_rate_config(struct rc_command *self, struct rate_config *rates);

//! Updates current rc rates from the current rc values. Inputs are in range [1000;2000] and should correspond to raw pwm values from rc receiver.
void rc_command_update(struct rc_command *self);

//! Get current stick value in range [-500;500], stick is only ROLL, PITCH, YAw
int16_t rc_command_axis(struct rc_command *self, uint8_t stick);

//! Get current angle command in decidegrees, range [-450;450]
int16_t rc_command_angle(struct rc_command *self, uint8_t stick);

//! Get rate command in degrees per second (with rate config applied), range [-3625;3625]
int16_t rc_command_rate(struct rc_command *self, uint8_t stick);

//! Get current throttle value in range [-500;500]
static inline int16_t rc_command_throttle(struct rc_command *self) { return self->throttle; }

//! Get current tpa value. Range [0;100]
static inline uint8_t rc_command_tpa(struct rc_command *self){ return self->tpa; } // TODO: tpa should probably not be here. This is just for now.

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

#include "system_calls.h"
#include "../config/mixer.h"
#include "../config/rc_controls.h"
#include "../config/rx.h"

#include "../common/filter.h"
#include "../common/maths.h"

struct config;
/*
#if USE_QUAD_MIXER_ONLY == 1
#define MAX_SUPPORTED_SERVOS 1
#else
#define MAX_SUPPORTED_SERVOS 8
#endif
#define MAX_SERVO_RULES (2 * MAX_SUPPORTED_SERVOS)
#define MAX_SERVO_SPEED UINT8_MAX
#define MAX_SERVO_BOXES 3
*/
// TODO: revise usage of MAX_SUPPORTED_MOTORS to see if it is wrong to define it to 12
/*
#if USE_QUAD_MIXER_ONLY == 1
#define MAX_SUPPORTED_MOTORS 4

#elif defined(TARGET_MOTOR_COUNT)
#define MAX_SUPPORTED_MOTORS TARGET_MOTOR_COUNT

#else
#define MAX_SUPPORTED_MOTORS 12
#endif
*/

#define YAW_JUMP_PREVENTION_LIMIT_LOW 80
#define YAW_JUMP_PREVENTION_LIMIT_HIGH 500

struct mixer {
	int16_t input[MIXER_INPUT_COUNT];

	// TODO: gimbal stuff should be above mixer code not part of it
	// move when we have refactored gimbal code
	//int16_t gimbal_angles[3];

	uint8_t flags;

	//! output count in current configuration being used by the mixer
	uint8_t motorCount;
	uint8_t servoCount;
	uint8_t ruleCount;

	bool motorLimitReached;

	//! currently used mixer mode
	uint8_t mode;

	struct mixer_rule_def active_rules[MIXER_MAX_RULES];

	//! output offset, min and max for motors
	int16_t midthrottle, minthrottle, maxthrottle;

	biquad_t servoFilterState[MAX_SUPPORTED_SERVOS];

	// TODO: mixer should not need so many configs. Need to factor out control logic out of the mixer!
	const struct config *config;

	const struct system_calls_pwm *pwm;
};

//! initializes a mixer struct
void mixer_init(struct mixer *self, const struct config *config, const struct system_calls_pwm *pwm);

//! inputs a command to one of the input channels of the mixer
void mixer_input_command(struct mixer *self, mixer_input_t i, int16_t value);

//! calculates outputs from all mixer inputs and mixing rules
void mixer_update(struct mixer *self);

//! puts mixer into armed state so that outputs are calculated (TODO: this should probably be placed outside of the mixer!)
void mixer_enable_armed(struct mixer *self, bool on);

//! tests if any of the motors have reached their limit (usually maxthrottle)
bool mixer_motor_limit_reached(struct mixer *self);

//! sets throttle range of the mixer (can be used to set 3d throttle range too)
void mixer_set_throttle_range(struct mixer *self, int16_t mid, int16_t min, int16_t max);
/*
//! returns a value of specified servo channel (id 0 is the first servo)
uint16_t mixer_get_servo_value(struct mixer *self, uint8_t id);

//! returns a value of specified motor channel (id 0 is the first motor)
uint16_t mixer_get_motor_value(struct mixer *self, uint8_t id);
*/
//! returns total number of motors that are being actively mixed by the mixer as part of current profile
uint8_t mixer_get_motor_count(struct mixer *self);

//! returns total number of servos that are being actively mxier by the mixer as part of current profile
uint8_t mixer_get_servo_count(struct mixer *self);

// TODO: remove these mixer loading/saving methods once user interface has been changed to the new mixer format

//! saves motor mixer into cleanflight motor mixer format
//int mixer_save_motor_mixer(struct mixer *self, struct motor_mixer *output);

//! loads a set of motor mixing rules from cleanflight format into current ruleset
//void mixer_load_motor_mixer(struct mixer *self, const struct motor_mixer *motors);

//! save servo mixer into cleanflight servo mixer format (sets some fields to defaults)
//int mixer_save_servo_mixer(struct mixer *self, struct servo_mixer *output);

//! loads servo mixing rules from cleanflight format into internal format
//void mixer_load_servo_mixer(struct mixer *self, const struct servo_mixer *servos);

//! clears all mixing rules
void mixer_clear_rules(struct mixer *self);


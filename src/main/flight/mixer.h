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

#include "anglerate.h"

// TODO: motor_and_servo probably does not belong in io folder..
#include "io/rc_controls.h"
#include "../config/mixer.h"

// TODO: this is very bad way so remove this later once refactoring is done.
extern struct mixer default_mixer;

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

// the following few structures are cleanflight config for motor and servo mixers which is also currently used for custom mixers
#define CHANNEL_FORWARDING_DISABLED (uint8_t)0xff

//! Configuration for each servo which is editable by the user
struct servo_config {
    int16_t min;                            //!< Minimum pwm value that is sent to the servo
    int16_t max;                            //!< Maximum pwm value that is sent to the servo
    int16_t middle;                         //!< PWM value that is sent to center the servo (usually 1500)
    int8_t rate;                            //!< range [-125;+125] ; can be used to adjust a rate 0-125% and a direction
    uint8_t angleAtMin;                     //!< range [0;180] the measured angle in degrees from the middle when the servo is at the 'min' value.
    uint8_t angleAtMax;                     //!< range [0;180] the measured angle in degrees from the middle when the servo is at the 'max' value.
    int8_t forwardFromChannel;              //!< RX channel index, 0 based.  See CHANNEL_FORWARDING_DISABLED
    uint32_t reversedSources;               //!< the direction of servo movement for each input source of the servo mixer, bit set=inverted
} __attribute__ ((__packed__));

struct servo_profile {
    struct servo_config servoConf[MAX_SUPPORTED_SERVOS];
};

/**
 * Mixer input commands that command the mixer to either spin or translate the frame in specific direction.
 * All movement is in body frame and movements that are not physically supported by the frame will be ignored.
 */
typedef enum {
	MIXER_INPUT_GROUP_FC = 0,
	MIXER_INPUT_G0_ROLL = 0,	//!< flight control angular rate for x axis
	MIXER_INPUT_G0_PITCH,		//!< flight control angular rate for y axis
	MIXER_INPUT_G0_YAW,			//!< flight control angular rate for z axis
	MIXER_INPUT_G0_THROTTLE,	//!< flight control throttle
	MIXER_INPUT_G0_FLAPS,		//!< flight control flaps
	MIXER_INPUT_G0_SPOILERS,	//!< flight control spoilers
	MIXER_INPUT_G0_AIRBREAKS,	//!< flight control airbreaks
	MIXER_INPUT_G0_LANDINGGEAR, //!< flight control landing gear
	MIXER_INPUT_GROUP_1 = (1 << 3),
	MIXER_INPUT_G1_ROLL = MIXER_INPUT_GROUP_1,
	MIXER_INPUT_G1_PITCH,
	MIXER_INPUT_G1_YAW,
	MIXER_INPUT_GROUP_GIMBAL = (2 << 3),
	MIXER_INPUT_G2_GIMBAL_ROLL = MIXER_INPUT_GROUP_GIMBAL, //!< gimbal roll
	MIXER_INPUT_G2_GIMBAL_PITCH,	//!< gimbal pitch
	MIXER_INPUT_G2_GIMBAL_YAW,		//!< gimbal yaw
	MIXER_INPUT_G2_GIMBAL_SHUTTER,	//!< gimbal shutter
	MIXER_INPUT_GROUP_RC = (3 << 3),
	MIXER_INPUT_G3_RC_ROLL = MIXER_INPUT_GROUP_RC, //!< rc roll input
	MIXER_INPUT_G3_RC_PITCH,
	MIXER_INPUT_G3_RC_YAW,
	MIXER_INPUT_G3_RC_THROTTLE,
	MIXER_INPUT_G3_RC_MODE,
	MIXER_INPUT_G3_RC_AUX1,
	MIXER_INPUT_G3_RC_AUX2,
	MIXER_INPUT_G3_RC_AUX3,
	MIXER_INPUT_GROUP_MOTOR_PASSTHROUGH = (4 << 3),
	MIXER_INPUT_G4_M1 = MIXER_INPUT_GROUP_MOTOR_PASSTHROUGH,
	MIXER_INPUT_G4_M2,
	MIXER_INPUT_G4_M3,
	MIXER_INPUT_G4_M4,
	MIXER_INPUT_G4_M5,
	MIXER_INPUT_G4_M6,
	MIXER_INPUT_G4_M7,
	MIXER_INPUT_G4_M8,
	MIXER_INPUT_COUNT
} mixer_input_t;

//! Mixer output channels
typedef enum {
	MIXER_OUTPUT_MOTORS = 0,
	MIXER_OUTPUT_M1 = MIXER_OUTPUT_MOTORS,
	MIXER_OUTPUT_M2,
	MIXER_OUTPUT_M3,
	MIXER_OUTPUT_M4,
	MIXER_OUTPUT_M5,
	MIXER_OUTPUT_M6,
	MIXER_OUTPUT_M7,
	MIXER_OUTPUT_M8,
	MIXER_MAX_MOTORS = MIXER_OUTPUT_M8 - MIXER_OUTPUT_MOTORS + 1,
	MIXER_OUTPUT_SERVOS = MIXER_OUTPUT_M8 + 1,
	MIXER_OUTPUT_S1 = MIXER_OUTPUT_SERVOS,
	MIXER_OUTPUT_S2,
	MIXER_OUTPUT_S3,
	MIXER_OUTPUT_S4,
	MIXER_OUTPUT_S5,
	MIXER_OUTPUT_S6,
	MIXER_OUTPUT_S7,
	MIXER_OUTPUT_S8,
	MIXER_MAX_SERVOS = MIXER_OUTPUT_S8 - MIXER_OUTPUT_SERVOS + 1,
	MIXER_OUTPUT_COUNT = MIXER_OUTPUT_S8 + 1
} mixer_output_t;

//! sets mixer frame configuration
typedef enum {
    MIXER_TRI = 1,
    MIXER_QUADP = 2,
    MIXER_QUADX = 3,
    MIXER_BICOPTER = 4,
    MIXER_GIMBAL = 5,
    MIXER_Y6 = 6,
    MIXER_HEX6 = 7,
    MIXER_FLYING_WING = 8,
    MIXER_Y4 = 9,
    MIXER_HEX6X = 10,
    MIXER_OCTOX8 = 11,
    MIXER_OCTOFLATP = 12,
    MIXER_OCTOFLATX = 13,
    MIXER_AIRPLANE = 14,        // airplane / singlecopter / dualcopter (not yet properly supported)
    MIXER_HELI_120_CCPM = 15,
    MIXER_HELI_90_DEG = 16,
    MIXER_VTAIL4 = 17,
    MIXER_HEX6H = 18,
    MIXER_PPM_TO_SERVO = 19,    // PPM -> servo relay
    MIXER_DUALCOPTER = 20,
    MIXER_SINGLECOPTER = 21,
    MIXER_ATAIL4 = 22,
    MIXER_QUADX_TILT1 = 23,
    MIXER_QUADX_TILT2 = 24,
    MIXER_CUSTOM,
    MIXER_CUSTOM_AIRPLANE,
    MIXER_CUSTOM_TRI,
	MIXER_MODE_COUNT,
} mixer_mode_t;

// TODO: all mixers should actually be stored in the client and we should only be supporting custom mixers (and possibly a default)

//! general mixer settings
struct mixer_config {
    uint8_t mixerMode;				//!< one of the mixer_mode_t values
    uint8_t pid_at_min_throttle;	//!< when enabled pids are used at minimum throttle (valid values 0 or 1)
	int8_t yaw_motor_direction;		//!< allows reversing yaw direction (values -1 or 1)
    uint16_t yaw_jump_prevention_limit; //!< make limit configurable (original fixed value was 100)
#ifdef USE_SERVOS
    uint8_t tri_unarmed_servo;		//!< send tail servo correction pulses even when unarmed
    float servo_lowpass_freq;		//!< lowpass servo filter frequency selection; 1/1000ths of loop freq
    int8_t servo_lowpass_enable;	//!< enable/disable lowpass filter for servo output
#endif
};

//! mixer rule definition in new format
struct mixer_rule_def {
	uint8_t output;
	uint8_t input;
	int16_t scale;
} __attribute__((packed));

#define MIXER_MAX_RULES 48

struct mixer {
	int16_t input[MIXER_INPUT_COUNT];
	int16_t output[MIXER_OUTPUT_COUNT];

	// TODO: gimbal stuff should be above mixer code not part of it
	// move when we have refactored gimbal code
	//int16_t gimbal_angles[3];

	uint8_t flags;

	//! output count in current configuration being used by the mixer
	uint8_t motorCount;
	uint8_t servoCount;
	uint8_t ruleCount;

	bool motorLimitReached;

	struct mixer_rule_def active_rules[MIXER_MAX_RULES];

	//! output offset, min and max for motors
	int16_t midthrottle, minthrottle, maxthrottle;

	biquad_t servoFilterState[MAX_SUPPORTED_SERVOS];

	// TODO: mixer should not need so many configs. Need to factor out control logic out of the mixer!
	struct mixer_config *mixer_config;
	struct motor_3d_config *motor_3d_config;
	motorAndServoConfig_t *motor_servo_config;
	rxConfig_t *rx_config;
	rcControlsConfig_t *rc_controls_config;
	struct servo_config *servo_config;
};

//! initializes a mixer struct
void mixer_init(struct mixer *self,
	struct mixer_config *mixer_config,
	struct motor_3d_config *mixer_3d_config,
	motorAndServoConfig_t *motor_servo_config,
	rxConfig_t *rx_config,
	rcControlsConfig_t *rc_controls_config,
	struct servo_config *servo_config,
	struct motor_mixer *custom_mixers,
	uint8_t count);

//! inputs a command to one of the input channels of the mixer
void mixer_input_command(struct mixer *self, mixer_input_t i, int16_t value);

//! loads a mixer preset into the current ruleset
void mixer_load_preset(struct mixer *self, mixer_mode_t preset);

//! calculates outputs from all mixer inputs and mixing rules
void mixer_update(struct mixer *self);

//! puts mixer into armed state so that outputs are calculated (TODO: this should probably be placed outside of the mixer!)
void mixer_enable_armed(struct mixer *self, bool on);

//! tests if any of the motors have reached their limit (usually maxthrottle)
bool mixer_motor_limit_reached(struct mixer *self);

//! sets throttle range of the mixer (can be used to set 3d throttle range too)
void mixer_set_throttle_range(struct mixer *self, int16_t mid, int16_t min, int16_t max);

//! returns a value of specified servo channel (id 0 is the first servo)
uint16_t mixer_get_servo_value(struct mixer *self, uint8_t id);

//! returns a value of specified motor channel (id 0 is the first motor)
uint16_t mixer_get_motor_value(struct mixer *self, uint8_t id);

//! returns total number of motors that are being actively mixed by the mixer as part of current profile
uint8_t mixer_get_motor_count(struct mixer *self);

//! returns total number of servos that are being actively mxier by the mixer as part of current profile
uint8_t mixer_get_servo_count(struct mixer *self);

// TODO: remove these mixer loading/saving methods once user interface has been changed to the new mixer format

//! saves motor mixer into cleanflight motor mixer format
int mixer_save_motor_mixer(struct mixer *self, struct motor_mixer *output);

//! loads a set of motor mixing rules from cleanflight format into current ruleset
void mixer_load_motor_mixer(struct mixer *self, const struct motor_mixer *motors);

//! save servo mixer into cleanflight servo mixer format (sets some fields to defaults)
int mixer_save_servo_mixer(struct mixer *self, struct servo_mixer *output);

//! loads servo mixing rules from cleanflight format into internal format
void mixer_load_servo_mixer(struct mixer *self, const struct servo_mixer *servos);

//! clears all mixing rules
void mixer_clear_rules(struct mixer *self);


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

#define MAX_SUPPORTED_MOTORS 12
#define MAX_SUPPORTED_SERVOS 8

#define MAX_SERVO_RULES (2 * MAX_SUPPORTED_SERVOS)
#define MAX_SERVO_SPEED UINT8_MAX
#define MAX_SERVO_BOXES 3

#define DEFAULT_SERVO_MIN 1000
#define DEFAULT_SERVO_MIDDLE 1500
#define DEFAULT_SERVO_MAX 2000
#define DEFAULT_SERVO_MIN_ANGLE 45
#define DEFAULT_SERVO_MAX_ANGLE 45

//! Cleanflight servo mixer definition.
struct servo_mixer {
	uint8_t targetChannel;                  //!< servo that receives the output of the rule
	uint8_t inputSource;                    //!< input channel for this rule
	int8_t rate;                            //!< range [-125;+125] ; can be used to adjust a rate 0-125% and a direction
	uint8_t speed;                          //!< reduces the speed of the rule, 0=unlimited speed
	int8_t min;                             //!< lower bound of rule range [0;100]% of servo max-min
	int8_t max;                             //!< lower bound of rule range [0;100]% of servo max-min
	uint8_t box;                            //!< active rule if box is enabled, range [0;3], 0=no box, 1=BOXSERVO1, 2=BOXSERVO2, 3=BOXSERVO3
} __attribute__((packed));

//! Cleanflight motor mixer definition used for custom mixers.
struct motor_mixer {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} __attribute__((packed));

//! 3d mode mixer settings (used when mixer_enable_3d_mode is called with true)
struct motor_3d_config {
    uint16_t deadband3d_low;                // min 3d value
    uint16_t deadband3d_high;               // max 3d value
    uint16_t neutral3d;                     // center 3d value
};

typedef struct motorAndServoConfig_s {

    // PWM values, in milliseconds, common range is 1000-2000 (1 to 2ms)
    uint16_t minthrottle;                   // Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
    uint16_t maxthrottle;                   // This is the maximum value for the ESCs at full power this value can be increased up to 2000
    uint16_t mincommand;                    // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
    uint16_t servoCenterPulse;              // This is the value for servos when they should be in the middle. e.g. 1500.

    uint16_t motor_pwm_rate;                // The update rate of motor outputs (50-498Hz)
    uint16_t servo_pwm_rate;                // The update rate of servo outputs (50-498Hz)
} motorAndServoConfig_t;

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


// TODO: custom servo and motor mixers should use mixer_rule_def instead. Remove the above defs once we are using new structures!

PG_DECLARE_ARR(struct motor_mixer, MAX_SUPPORTED_MOTORS, customMotorMixer);
PG_DECLARE(struct mixer_config, mixerConfig);
PG_DECLARE(struct motor_3d_config, motor3DConfig);
PG_DECLARE(motorAndServoConfig_t, motorAndServoConfig);
PG_DECLARE_ARR(struct servo_mixer, MAX_SERVO_RULES, customServoMixer);
PG_DECLARE_PROFILE(struct servo_profile, servoProfile);



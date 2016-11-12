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

#define MAX_SUPPORTED_MOTORS 12
#define MAX_SUPPORTED_SERVOS 8

#define MAX_SERVO_RULES (2 * MAX_SUPPORTED_SERVOS)
#define MAX_SERVO_SPEED UINT8_MAX
#define MAX_SERVO_BOXES 3

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

// TODO: custom servo and motor mixers should use mixer_rule_def instead. Remove the above defs once we are using new structures!

PG_DECLARE_ARR(struct motor_mixer, MAX_SUPPORTED_MOTORS, customMotorMixer);
PG_DECLARE(struct mixer_config, mixerConfig);
PG_DECLARE(struct motor_3d_config, motor3DConfig);
PG_DECLARE(motorAndServoConfig_t, motorAndServoConfig);
PG_DECLARE_ARR(struct servo_mixer, MAX_SERVO_RULES, customServoMixer);
PG_DECLARE_PROFILE(struct servo_profile, servoProfile);



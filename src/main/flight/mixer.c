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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <platform.h>
#include "debug.h"

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/parameter_group.h"

#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/system.h"

#include "rx/rx.h"

#include "io/gimbal.h"
#include "io/motor_and_servo.h"
#include "io/rc_controls.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/anglerate.h"
#include "flight/imu.h"

#include "config/parameter_group_ids.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/feature.h"
#include "config/config_reset.h"

#include "servos.h"


/* QuadX
4CW   2CCW
   \ /
    X
   / \
3CCW  1CW
*/
static const struct motor_mixer mixerQuadX[] = {
//throttle,  roll, pitch,   yaw
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R  (M1)
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R (M2)
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L  (M3)
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L (M4)
};

#ifndef USE_QUAD_MIXER_ONLY
/* QuadP
    4CW
     |
3CCW-+-2CCW
     |
    1CW
*/
static const struct motor_mixer mixerQuadP[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR
    { 1.0f, -1.0f,  0.0f,  1.0f },          // RIGHT
    { 1.0f,  1.0f,  0.0f,  1.0f },          // LEFT
    { 1.0f,  0.0f, -1.0f, -1.0f },          // FRONT
};

/* Vtail4
4CCW-----2CW
     ||
     ||
3CW\ || /1CCW
    \||/
*/
static const struct motor_mixer mixerVtail4[] = {
    { 1.0f, -0.58f,  0.58f,  1.0f },        // REAR_R
    { 1.0f, -0.46f, -0.39f, -0.5f },        // FRONT_R
    { 1.0f,  0.58f,  0.58f, -1.0f },        // REAR_L
    { 1.0f,  0.46f, -0.39f,  0.5f },        // FRONT_L
};

/* Atail4
 4CW----2CCW
     ||
     ||
     /\
3CCW/  \1CW
*/
static const struct motor_mixer mixerAtail4[] = {
    { 1.0f, -0.58f,  0.58f, -1.0f },        // REAR_R
    { 1.0f, -0.46f, -0.39f,  0.5f },        // FRONT_R
    { 1.0f,  0.58f,  0.58f,  1.0f },        // REAR_L
    { 1.0f,  0.46f, -0.39f, -0.5f },        // FRONT_L
};

/* Y4
 4CW----2CCW
     ||
     ||
    1CW
    3CCW
*/
static const struct motor_mixer mixerY4[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_TOP CW
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R CCW
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_BOTTOM CCW
    { 1.0f,  1.0f, -1.0f,  0.0f },          // FRONT_L CW
};

#if (MAX_SUPPORTED_MOTORS >= 6)
static const struct motor_mixer mixerY6[] = {
    { 1.0f,  0.0f,  1.333333f,  1.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f, -1.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f, -1.0f },     // LEFT
    { 1.0f,  0.0f,  1.333333f, -1.0f },     // UNDER_REAR
    { 1.0f, -1.0f, -0.666667f,  1.0f },     // UNDER_RIGHT
    { 1.0f,  1.0f, -0.666667f,  1.0f },     // UNDER_LEFT
};

static const struct motor_mixer mixerHex6H[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f,  0.0f,  0.0f,  0.0f },          // RIGHT
    { 1.0f,  0.0f,  0.0f,  0.0f },          // LEFT
};

static const struct motor_mixer mixerHex6P[] = {
    { 1.0f, -0.866025f,  0.5f,  1.0f },     // REAR_R
    { 1.0f, -0.866025f, -0.5f, -1.0f },     // FRONT_R
    { 1.0f,  0.866025f,  0.5f,  1.0f },     // REAR_L
    { 1.0f,  0.866025f, -0.5f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f,      -1.0f,  1.0f },     // FRONT
    { 1.0f,  0.0f,       1.0f, -1.0f },     // REAR
};

static const struct motor_mixer mixerHex6X[] = {
    { 1.0f, -0.5f,  0.866025f,  1.0f },     // REAR_R
    { 1.0f, -0.5f, -0.866025f,  1.0f },     // FRONT_R
    { 1.0f,  0.5f,  0.866025f, -1.0f },     // REAR_L
    { 1.0f,  0.5f, -0.866025f, -1.0f },     // FRONT_L
    { 1.0f, -1.0f,  0.0f,      -1.0f },     // RIGHT
    { 1.0f,  1.0f,  0.0f,       1.0f },     // LEFT
};
#endif

#if (MAX_SUPPORTED_MOTORS >= 8)
static const struct motor_mixer mixerOctoX8[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f,  1.0f,  1.0f },          // UNDER_REAR_R
    { 1.0f, -1.0f, -1.0f, -1.0f },          // UNDER_FRONT_R
    { 1.0f,  1.0f,  1.0f, -1.0f },          // UNDER_REAR_L
    { 1.0f,  1.0f, -1.0f,  1.0f },          // UNDER_FRONT_L
};

static const struct motor_mixer mixerOctoFlatP[] = {
    { 1.0f,  0.707107f, -0.707107f,  1.0f },// FRONT_L
    { 1.0f, -0.707107f, -0.707107f,  1.0f },// FRONT_R
    { 1.0f, -0.707107f,  0.707107f,  1.0f },// REAR_R
    { 1.0f,  0.707107f,  0.707107f,  1.0f },// REAR_L
    { 1.0f,  0.0f,      -1.0f,      -1.0f },// FRONT
    { 1.0f, -1.0f,       0.0f,      -1.0f },// RIGHT
    { 1.0f,  0.0f,       1.0f,      -1.0f },// REAR
    { 1.0f,  1.0f,       0.0f,      -1.0f },// LEFT
};

static const struct motor_mixer mixerOctoFlatX[] = {
    { 1.0f,  1.0f,      -0.414178f,  1.0f },// MIDFRONT_L
    { 1.0f, -0.414178f, -1.0f,       1.0f },// FRONT_R
    { 1.0f, -1.0f,       0.414178f,  1.0f },// MIDREAR_R
    { 1.0f,  0.414178f,  1.0f,       1.0f },// REAR_L
    { 1.0f,  0.414178f, -1.0f,      -1.0f },// FRONT_L
    { 1.0f, -1.0f,      -0.414178f, -1.0f },// MIDFRONT_R
    { 1.0f, -0.414178f,  1.0f,      -1.0f },// REAR_R
    { 1.0f,  1.0f,       0.414178f, -1.0f },// MIDREAR_L
};
#endif

static const struct motor_mixer mixerSingleProp[] = {
    { 1.0f,  0.0f,  0.0f, 0.0f },
};

static const struct motor_mixer mixerDualcopter[] = {
    { 1.0f,  0.0f,  0.0f, -1.0f },          // LEFT
    { 1.0f,  0.0f,  0.0f,  1.0f },          // RIGHT
};

static const struct motor_mixer mixerBicopter[] = {
    { 1.0f,  1.0f,  0.0f,  0.0f },          // LEFT
    { 1.0f, -1.0f,  0.0f,  0.0f },          // RIGHT
};

static const struct motor_mixer mixerTricopter[] = {
    { 1.0f,  0.0f,  1.333333f,  0.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f,  0.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f,  0.0f },     // LEFT
};

// Keep synced with mixerMode_e
const struct mixer_mode mixers[] = {
    // motors, use servo, motor mixer
    { 0, false, NULL },                // entry 0
    { 3, true,  mixerTricopter },      // MIXER_TRI
    { 4, false, mixerQuadP },          // MIXER_QUADP
    { 4, false, mixerQuadX },          // MIXER_QUADX
    { 2, true,  mixerBicopter },       // MIXER_BICOPTER
    { 0, true,  NULL },                // * MIXER_GIMBAL
#if (MAX_SUPPORTED_MOTORS >= 6)
    { 6, false, mixerY6 },             // MIXER_Y6
    { 6, false, mixerHex6P },          // MIXER_HEX6
#else
    { 6, false, NULL },                // MIXER_Y6
    { 6, false, NULL },                // MIXER_HEX6
#endif
    { 1, true,  mixerSingleProp },     // * MIXER_FLYING_WING
    { 4, false, mixerY4 },             // MIXER_Y4
#if (MAX_SUPPORTED_MOTORS >= 6)
    { 6, false, mixerHex6X },          // MIXER_HEX6X
#else
    { 6, false, NULL },                // MIXER_HEX6X
#endif
#if (MAX_SUPPORTED_MOTORS >= 8)
    { 8, false, mixerOctoX8 },         // MIXER_OCTOX8
    { 8, false, mixerOctoFlatP },      // MIXER_OCTOFLATP
    { 8, false, mixerOctoFlatX },      // MIXER_OCTOFLATX
#else
    { 8, false, NULL },                // MIXER_OCTOX8
    { 8, false, NULL },                // MIXER_OCTOFLATP
    { 8, false, NULL },                // MIXER_OCTOFLATX
#endif
    { 1, true,  mixerSingleProp },     // * MIXER_AIRPLANE
    { 0, true,  NULL },                // * MIXER_HELI_120_CCPM
    { 0, true,  NULL },                // * MIXER_HELI_90_DEG
    { 4, false, mixerVtail4 },         // MIXER_VTAIL4
#if (MAX_SUPPORTED_MOTORS >= 6)
    { 6, false, mixerHex6H },          // MIXER_HEX6H
#else
    { 6, false, NULL },                // MIXER_HEX6H
#endif
    { 0, true,  NULL },                // * MIXER_PPM_TO_SERVO
    { 2, true,  mixerDualcopter },     // MIXER_DUALCOPTER
    { 1, true,  NULL },                // MIXER_SINGLECOPTER
    { 4, false, mixerAtail4 },         // MIXER_ATAIL4
    { 4, true, mixerQuadX },           // MIXER_QUADX_TILT1
    { 4, true, mixerQuadX },            // MIXER_QUADX_TILT2
    { 0, false, NULL },                // MIXER_CUSTOM
    { 2, true,  NULL },                // MIXER_CUSTOM_AIRPLANE
    { 3, true,  NULL },                // MIXER_CUSTOM_TRI
};
#endif


// TODO: fix the self reference and remove all externs 
void mixer_init(struct mixer *self, struct motor_mixer *initialCustomMixers, uint8_t count){
	(void)count; 

	// set quadx mixer by default
	self->motorCount = 4; 
	for (int i = 0; i < self->motorCount; i++) {
        self->currentMixer[i] = mixerQuadX[i];
    }
    mixer_reset_disarmed_pwm_values(self);

    self->customMixers = initialCustomMixers;
}

//#if !defined(USE_SERVOS) || defined(USE_QUAD_MIXER_ONLY)
// TODO: what is this useful for exactly? 
/*
void mixer_use_pwmio_config(struct mixer *self, struct pwmIOConfiguration_s *pwmIOConfiguration)
{
    UNUSED(pwmIOConfiguration);
	self->motorCount = 4; 
    uint8_t i;
    for (i = 0; i < self->motorCount; i++) {
        self->currentMixer[i] = mixerQuadX[i];
    }
    mixer_reset_disarmed_pwm_values(self);
}
*/
//#endif


#ifndef USE_QUAD_MIXER_ONLY
// TODO: this method does not make sense. Change name. 
void mixer_load_motor_mixer(struct mixer *self, int index, struct motor_mixer *customMixers)
{
	(void)self; 
    int i;

    // we're 1-based
    index++;
    // clear existing
    for (i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        customMixers[i].throttle = 0.0f;

    // do we have anything here to begin with?
    if (mixers[index].motor != NULL) {
        for (i = 0; i < mixers[index].motorCount; i++)
            customMixers[i] = mixers[index].motor[i];
    }
}

#endif

void mixer_reset_disarmed_pwm_values(struct mixer *self)
{
    // set disarmed motor values
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        self->motor_disarmed[i] = feature(FEATURE_3D) ? motor3DConfig()->neutral3d : motorAndServoConfig()->mincommand;
}

void mixer_write_pwm(struct mixer *self)
{
    uint8_t i;

    for (i = 0; i < self->motorCount; i++)
        pwmWriteMotor(i, self->motor[i]);


    if (feature(FEATURE_ONESHOT125)) {
        pwmCompleteOneshotMotorUpdate(self->motorCount);
    }
}

void mixer_set_all_motors_pwm(struct mixer *self, int16_t mc)
{
    uint8_t i;

    // Sends commands to all motors
    for (i = 0; i < self->motorCount; i++)
        self->motor[i] = mc;
    mixer_write_pwm(self);
}

void mixer_stop_motors(struct mixer *self)
{
    mixer_set_all_motors_pwm(self, feature(FEATURE_3D) ? motor3DConfig()->neutral3d : motorAndServoConfig()->mincommand);

	// TODO: remove this delay
    usleep(50000); // give the timers and ESCs a chance to react.
}

// TODO: all pwm functions need to be moved outside of mixer!
void mixer_stop_pwm_all_motors(struct mixer *self)
{
    pwmShutdownPulsesForAllMotors(self->motorCount);
}

static uint16_t mixConstrainMotorForFailsafeCondition(struct mixer *self, uint8_t motorIndex)
{
    return constrain(self->motor[motorIndex], motorAndServoConfig()->mincommand, motorAndServoConfig()->maxthrottle);
}

void mixer_update(struct mixer *self, const struct pid_controller_output *pid_axis){
    uint32_t i;
	int16_t axis[3] = {pid_axis->axis[0], pid_axis->axis[1], pid_axis->axis[2]}; 

    bool isFailsafeActive = failsafeIsActive();

    if (self->motorCount >= 4 && mixerConfig()->yaw_jump_prevention_limit < YAW_JUMP_PREVENTION_LIMIT_HIGH) {
        // prevent "yaw jump" during yaw correction
        axis[FD_YAW] = constrain(axis[FD_YAW], -mixerConfig()->yaw_jump_prevention_limit - ABS(rcCommand[YAW]), mixerConfig()->yaw_jump_prevention_limit + ABS(rcCommand[YAW]));
    }

    if (rcModeIsActive(BOXAIRMODE)) {
        // Initial mixer concept by bdoiron74 reused and optimized for Air Mode
        int16_t rollPitchYawMix[MAX_SUPPORTED_MOTORS];
        int16_t rollPitchYawMixMax = 0; // assumption: symetrical about zero.
        int16_t rollPitchYawMixMin = 0;

        // Find roll/pitch/yaw desired output
        for (i = 0; i < self->motorCount; i++) {
            rollPitchYawMix[i] =
                axis[FD_PITCH] * self->currentMixer[i].pitch +
                axis[FD_ROLL] * self->currentMixer[i].roll +
                -mixerConfig()->yaw_motor_direction * axis[FD_YAW] * self->currentMixer[i].yaw;

            if (rollPitchYawMix[i] > rollPitchYawMixMax) rollPitchYawMixMax = rollPitchYawMix[i];
            if (rollPitchYawMix[i] < rollPitchYawMixMin) rollPitchYawMixMin = rollPitchYawMix[i];
        }

        // Scale roll/pitch/yaw uniformly to fit within throttle range
        int16_t rollPitchYawMixRange = rollPitchYawMixMax - rollPitchYawMixMin;
        int16_t throttleRange, throttle;
        int16_t throttleMin, throttleMax;
        static int16_t throttlePrevious = 0;   // Store the last throttle direction for deadband transitions in 3D.

        // Find min and max throttle based on condition. Use rcData for 3D to prevent loss of power due to min_check
        if (feature(FEATURE_3D)) {
            if (!ARMING_FLAG(ARMED)) throttlePrevious = rxConfig()->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.

            if ((rc_get_channel_value(THROTTLE) <= (rxConfig()->midrc - rcControlsConfig()->deadband3d_throttle))) { // Out of band handling
                throttleMax = motor3DConfig()->deadband3d_low;
                throttleMin = motorAndServoConfig()->minthrottle;
                throttlePrevious = throttle = rc_get_channel_value(THROTTLE);
            } else if (rc_get_channel_value(THROTTLE) >= (rxConfig()->midrc + rcControlsConfig()->deadband3d_throttle)) { // Positive handling
                throttleMax = motorAndServoConfig()->maxthrottle;
                throttleMin = motor3DConfig()->deadband3d_high;
                throttlePrevious = throttle = rc_get_channel_value(THROTTLE);
            } else if ((throttlePrevious <= (rxConfig()->midrc - rcControlsConfig()->deadband3d_throttle)))  { // Deadband handling from negative to positive
                throttle = throttleMax = motor3DConfig()->deadband3d_low;
                throttleMin = motorAndServoConfig()->minthrottle;
            } else {  // Deadband handling from positive to negative
                throttleMax = motorAndServoConfig()->maxthrottle;
                throttle = throttleMin = motor3DConfig()->deadband3d_high;
            }
        } else {
            throttle = rcCommand[THROTTLE];
            throttleMin = motorAndServoConfig()->minthrottle;
            throttleMax = motorAndServoConfig()->maxthrottle;
        }

        throttleRange = throttleMax - throttleMin;

        if (rollPitchYawMixRange > throttleRange) {
            self->motorLimitReached = true;
            float mixReduction = (float) throttleRange / rollPitchYawMixRange;
            for (i = 0; i < self->motorCount; i++) {
                rollPitchYawMix[i] =  lrintf((float) rollPitchYawMix[i] * mixReduction);
            }
            // Get the maximum correction by setting throttle offset to center.
            throttleMin = throttleMax = throttleMin + (throttleRange / 2);
        } else {
            self->motorLimitReached = false;
            throttleMin = throttleMin + (rollPitchYawMixRange / 2);
            throttleMax = throttleMax - (rollPitchYawMixRange / 2);
        }

        // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
        // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
        for (i = 0; i < self->motorCount; i++) {
            self->motor[i] = rollPitchYawMix[i] + constrain(throttle * self->currentMixer[i].throttle, throttleMin, throttleMax);

            if (isFailsafeActive) {
                self->motor[i] = mixConstrainMotorForFailsafeCondition(self, i);
            } else if (feature(FEATURE_3D)) {
                if (throttlePrevious <= (rxConfig()->midrc - rcControlsConfig()->deadband3d_throttle)) {
                    self->motor[i] = constrain(self->motor[i], motorAndServoConfig()->minthrottle, motor3DConfig()->deadband3d_low);
                } else {
                    self->motor[i] = constrain(self->motor[i], motor3DConfig()->deadband3d_high, motorAndServoConfig()->maxthrottle);
                }
            } else {
                self->motor[i] = constrain(self->motor[i], motorAndServoConfig()->minthrottle, motorAndServoConfig()->maxthrottle);
            }
        }
    } else {
        // motors for non-servo mixes
        for (i = 0; i < self->motorCount; i++) {
            self->motor[i] =
                rcCommand[THROTTLE] * self->currentMixer[i].throttle +
                axis[FD_PITCH] * self->currentMixer[i].pitch +
                axis[FD_ROLL] * self->currentMixer[i].roll +
                -mixerConfig()->yaw_motor_direction * axis[FD_YAW] * self->currentMixer[i].yaw;
        }

        // Find the maximum motor output.
        int16_t maxMotor = self->motor[0];
        for (i = 1; i < self->motorCount; i++) {
            // If one motor is above the maxthrottle threshold, we reduce the value
            // of all motors by the amount of overshoot.  That way, only one motor
            // is at max and the relative power of each motor is preserved.
            if (self->motor[i] > maxMotor) {
                maxMotor = self->motor[i];
            }
        }

        int16_t maxThrottleDifference = 0;
        if (maxMotor > motorAndServoConfig()->maxthrottle) {
            maxThrottleDifference = maxMotor - motorAndServoConfig()->maxthrottle;
        }

		int16_t throttle = rc_get_channel_value(THROTTLE); 
        for (i = 0; i < self->motorCount; i++) {
            // this is a way to still have good gyro corrections if at least one motor reaches its max.
            self->motor[i] -= maxThrottleDifference;
			
            if (feature(FEATURE_3D)) {
                if (mixerConfig()->pid_at_min_throttle
                        || throttle <= rxConfig()->midrc - rcControlsConfig()->deadband3d_throttle
                        || throttle >= rxConfig()->midrc + rcControlsConfig()->deadband3d_throttle) {
                    if (rc_get_channel_value(THROTTLE) > rxConfig()->midrc) {
                        self->motor[i] = constrain(self->motor[i], motor3DConfig()->deadband3d_high, motorAndServoConfig()->maxthrottle);
                    } else {
                        self->motor[i] = constrain(self->motor[i], motorAndServoConfig()->mincommand, motor3DConfig()->deadband3d_low);
                    }
                } else {
                    if (throttle > rxConfig()->midrc) {
                        self->motor[i] = motor3DConfig()->deadband3d_high;
                    } else {
                        self->motor[i] = motor3DConfig()->deadband3d_low;
                    }
                }
            } else {
                if (isFailsafeActive) {
                    self->motor[i] = mixConstrainMotorForFailsafeCondition(self, i);
                } else {
                    // If we're at minimum throttle and FEATURE_MOTOR_STOP enabled,
                    // do not spin the motors.
                    self->motor[i] = constrain(self->motor[i], motorAndServoConfig()->minthrottle, motorAndServoConfig()->maxthrottle);
                    if (throttle < rxConfig()->mincheck) {
                        if (feature(FEATURE_MOTOR_STOP)) {
                            self->motor[i] = motorAndServoConfig()->mincommand;
                        } else if (mixerConfig()->pid_at_min_throttle == 0) {
                            self->motor[i] = motorAndServoConfig()->minthrottle;
                        }
                    }
                }
            }
        }
    }

    /* Disarmed for all mixers */
	// TODO: this should be moved higher up in this function. Need to evaluate effects of doing that. 
    if (!ARMING_FLAG(ARMED)) {
        for (i = 0; i < self->motorCount; i++) {
            self->motor[i] = self->motor_disarmed[i];
        }
    }

#if !defined(USE_QUAD_MIXER_ONLY) && defined(USE_SERVOS)
	mixer_update_servos(self, pid_axis);
#endif

}

void mixer_enable_3d_mode(struct mixer *self, bool on){
	self->mode3d = on;
}

void mixer_input_gimbal_angles(struct mixer *self, int16_t roll_dd, int16_t pitch_dd, int16_t yaw_dd){
	self->gimbal_angles[ROLL] = roll_dd;
	self->gimbal_angles[PITCH] = pitch_dd;
	self->gimbal_angles[YAW] = yaw_dd;
}

void mixer_set_motor_disarmed_pwm(struct mixer *self, uint8_t id, int16_t value){
	if(id >= self->motorCount) return; 
	self->motor_disarmed[id] = value; 
}

int16_t mixer_get_motor_disarmed_pwm(struct mixer *self, uint8_t id){
	if(id >= self->motorCount) return 0; 
	return self->motor_disarmed[id]; 
}

uint16_t mixer_get_motor_value(struct mixer *self, uint8_t id){
	if(id >= self->motorCount) return 0; 
	return self->motor[id]; 
}

bool mixer_motor_limit_reached(struct mixer *self){
	return self->motorLimitReached; 
}

uint8_t mixer_get_motor_count(struct mixer *self){
	return self->motorCount; 
}

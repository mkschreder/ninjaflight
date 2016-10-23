/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
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
#include "flight/pid.h"
#include "flight/imu.h"

#include "config/parameter_group_ids.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/feature.h"
#include "config/config_reset.h"

//#define MIXER_DEBUG
struct mixer default_mixer; 

PG_REGISTER_ARR(struct motor_mixer, MAX_SUPPORTED_MOTORS, customMotorMixer, PG_MOTOR_MIXER, 0);
PG_REGISTER_WITH_RESET_TEMPLATE(struct mixer_config, mixerConfig, PG_MIXER_CONFIG, 0);
PG_REGISTER_WITH_RESET_TEMPLATE(struct motor_3d_config, motor3DConfig, PG_MOTOR_3D_CONFIG, 0);

PG_RESET_TEMPLATE(struct motor_3d_config, motor3DConfig,
    .deadband3d_low = 1406,
    .deadband3d_high = 1514,
    .neutral3d = 1460,
);


#ifdef USE_SERVOS
PG_RESET_TEMPLATE(struct mixer_config, mixerConfig,
    .mixerMode = MIXER_QUADX,
    .pid_at_min_throttle = 1,
    .yaw_motor_direction = 1,
    .yaw_jump_prevention_limit = 200,

    .tri_unarmed_servo = 1,
    .servo_lowpass_freq = 400.0f,
);
#else
PG_RESET_TEMPLATE(struct mixer_config, mixerConfig,
    .mixerMode = MIXER_QUADX,
    .pid_at_min_throttle = 1,
    .yaw_motor_direction = 1,
    .yaw_jump_prevention_limit = 200,
);
#endif

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
	self->motorCount = 0; 
    self->customMixers = initialCustomMixers;
}

//#if !defined(USE_SERVOS) || defined(USE_QUAD_MIXER_ONLY)
// TODO: what is this useful for exactly? 
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
    delay(50); // give the timers and ESCs a chance to react.
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

// TODO: get rid of these and put them into a config
const float servo_angle_min = -45; 
const float servo_angle_max = 45; 

/*
static int16_t _tilt_radians_to_pwm(float angle_radians) {
    // normalize angle to range -PI to PI
    while (angle_radians > M_PIf)
        angle_radians -= M_PIf * 2;
    while (angle_radians < -M_PIf)
        angle_radians += M_PIf * 2;

    //remap input value (RX limit) to output value (Servo limit), also take into account eventual non-linearity of the two half range
    if (angle_radians > 0) {
        angle_radians = scaleRangef(angle_radians, 0, degreesToRadians(servo_angle_max), 0, 500);
    } else {
        angle_radians = scaleRangef(angle_radians, 0, -degreesToRadians(servo_angle_min), 0, -500);
    }

    //just to be sure it is in range, because float
    return constrain(angle_radians, -500, 500);
}
*/

static float _tilt_pwm_to_radians(int16_t channel) {
    //check input
    channel = constrain(channel, -500, 500);

    float angle;
    //take into account eventual non-linearity of the range
    if (channel > 0){
        angle = scaleRangef(channel, 0, 500, 0,  degreesToRadians(servo_angle_max) );
    }else{
        angle = scaleRangef(channel, 0, -500, 0, -degreesToRadians(servo_angle_min) );
    }

    return angle;
}

/*
 * return a float in range [-PI/2:+PI/2] witch represent the actual servo inclination wanted
 */
static float _mixer_calc_tilt_angle(struct mixer *self) {
    int16_t userInput = 0;
    uint8_t isFixedPitch = false;
	const float gearRatio = 100.0f; 
    //get wanted position of the tilting servo
	// TODO: if servo is enabled
    //if (rc_get_channel_value(PITCH) >= rxConfig()->midrc) {
        //userInput = rc_get_channel_value(PITCH) - 1500;
        //isFixedPitch = true;
    //} else {
        userInput = rcCommand[PITCH]; //use rcCommand so we get expo and deadband
    //}

    if ( !FLIGHT_MODE(ANGLE_MODE) && !FLIGHT_MODE(HORIZON_MODE) && !isFixedPitch) {

        //TODO: change this hardcoded value to something user can select; those affect the speed of the servo in rate mode
        //user input is from -500 to 500, we want to scale it from -10deg to +10deg
        float servoSpeed;
        if (userInput > 0) {
            servoSpeed = scaleRangef( userInput, 0, 500, 0, 0.017f );
        } else {
            servoSpeed = scaleRangef( userInput, 0, -500, 0, -0.017f);
        }

        self->lastServoAngleTilt += (servoSpeed * gearRatio) / 100.0f;

        // prevent overshot and overflow/windup
        self->lastServoAngleTilt = constrainf(self->lastServoAngleTilt, -degreesToRadians(servo_angle_min), degreesToRadians(servo_angle_max));

        return self->lastServoAngleTilt;
    } else {
        self->lastServoAngleTilt = 0; //reset

        return (_tilt_pwm_to_radians(userInput) * gearRatio) / 100.0f;
    }
	
	return 0; 
}

static void _mixer_mix_tilt(struct mixer *self) {
    float angleTilt = _mixer_calc_tilt_angle(self);
    float tmpCosine = cos_approx(angleTilt);
	// TODO: make this a config var
	static const bool thrust_compensation = true; 
	static const bool yaw_roll_compensation = true; 
	static const bool body_compensation = true; 
	int16_t liftoff_thrust = 0; 

    if (thrust_compensation) {
        // compensate the throttle because motor orientation
        float pitchToCompensate = angleTilt;

        float bodyPitch = degreesToRadians(attitude.values.pitch);
        if (body_compensation) {
            pitchToCompensate += bodyPitch;
        }

        pitchToCompensate = ABS(pitchToCompensate); //we compensate in the same way if up or down.

        if (pitchToCompensate > 0 && angleTilt + bodyPitch < M_PIf / 2) { //if there is something to compensate, and only from 0 to 90, otherwise it will push you into the ground
            uint16_t liftOffTrust = ((rxConfig()->maxcheck - rxConfig()->mincheck) * liftoff_thrust) / 100; //force this order so we don't need float!
            uint16_t liftOffLimit = ((rcCommand[THROTTLE] - (rxConfig()->maxcheck - rxConfig()->mincheck)) * 80) / 100; //we will artificially limit the trust compensation to 80% of remaining trust

            float tmp_cos_compensate = cos_approx(pitchToCompensate);
            if (tmp_cos_compensate != 0) { //it may be zero if the pitchToCOmpensate is 90°, also if it is very close due to float approximation.
                float compensation = liftOffTrust / tmp_cos_compensate; //absolute value because we want to increase power even if breaking

                if (compensation > 0) { //prevent overflow
                    rcCommand[THROTTLE] += (compensation < liftOffLimit) ? compensation : liftOffLimit;
                }
            }
        }
    }

    //compensate the roll and yaw because motor orientation
    if (yaw_roll_compensation) {

        // ***** quick and dirty compensation to test *****
        float rollCompensation = axisPID[ROLL] * tmpCosine;
        float rollCompensationInv = axisPID[ROLL] - rollCompensation;
        float yawCompensation = axisPID[YAW] * tmpCosine;
        float yawCompensationInv = axisPID[YAW] - yawCompensation;

        axisPID[ROLL] = yawCompensationInv + rollCompensation;
        axisPID[YAW] = yawCompensation + rollCompensationInv;
    }
}

void mixer_update(struct mixer *self)
{
    uint32_t i;

    bool isFailsafeActive = failsafeIsActive();

    if (self->motorCount >= 4 && mixerConfig()->yaw_jump_prevention_limit < YAW_JUMP_PREVENTION_LIMIT_HIGH) {
        // prevent "yaw jump" during yaw correction
        axisPID[FD_YAW] = constrain(axisPID[FD_YAW], -mixerConfig()->yaw_jump_prevention_limit - ABS(rcCommand[YAW]), mixerConfig()->yaw_jump_prevention_limit + ABS(rcCommand[YAW]));
    }

    if (rcModeIsActive(BOXAIRMODE)) {
        // Initial mixer concept by bdoiron74 reused and optimized for Air Mode
        int16_t rollPitchYawMix[MAX_SUPPORTED_MOTORS];
        int16_t rollPitchYawMixMax = 0; // assumption: symetrical about zero.
        int16_t rollPitchYawMixMin = 0;

        // Find roll/pitch/yaw desired output
        for (i = 0; i < self->motorCount; i++) {
            rollPitchYawMix[i] =
                axisPID[FD_PITCH] * self->currentMixer[i].pitch +
                axisPID[FD_ROLL] * self->currentMixer[i].roll +
                -mixerConfig()->yaw_motor_direction * axisPID[FD_YAW] * self->currentMixer[i].yaw;

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
                axisPID[FD_PITCH] * self->currentMixer[i].pitch +
                axisPID[FD_ROLL] * self->currentMixer[i].roll +
                -mixerConfig()->yaw_motor_direction * axisPID[FD_YAW] * self->currentMixer[i].yaw;
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

	if(mixerConfig()->mixerMode == MIXER_QUADX_TILT1 || mixerConfig()->mixerMode == MIXER_QUADX_TILT2){
		_mixer_mix_tilt(self); 
	}

    /* Disarmed for all mixers */
	// TODO: this should be moved higher up in this function. Need to evaluate effects of doing that. 
    if (!ARMING_FLAG(ARMED)) {
        for (i = 0; i < self->motorCount; i++) {
            self->motor[i] = self->motor_disarmed[i];
        }
    }

    // motor outputs are used as sources for servo mixing, so motors must be calculated before servos.

#if !defined(USE_QUAD_MIXER_ONLY) && defined(USE_SERVOS)
    mixer_update_servos(self);
#endif
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

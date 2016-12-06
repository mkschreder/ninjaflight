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

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

extern "C" {
    #include "debug.h"

    #include <platform.h>
    #include "build_config.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/filter.h"

	#include "config/gimbal.h"

    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "drivers/pwm_mapping.h"
    #include "drivers/gyro_sync.h"

    #include "sensors/acceleration.h"

    #include "rx/rx.h"
    #include "flight/anglerate.h"
    #include "flight/mixer.h"

    #include "config/config.h"

    //void mixerUsePWMIOConfiguration(struct mixer *self, pwmIOConfiguration_t *pwmIOConfiguration);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

/**
 * @page MIXER
 * @ingroup MIXER
 *
 * Mixer Unit Tests
 * ----------------
 *
 * This is a summary of automatic tests that are done against the mixer module
 * to guarantee that the module behaves according to the requirements set forth
 * below.
 */

struct mixer default_mixer; 

// input
#define TEST_RC_MID 1500

// output
#define TEST_MIN_COMMAND 1000
#define TEST_SERVO_MID 1500

void resetRX(){
	mock_rc_pwm[ROLL] = 1500;
	mock_rc_pwm[PITCH] = 1500;
	mock_rc_pwm[YAW] = 1500;
	mock_rc_pwm[THROTTLE] = 1000;
}

TEST(FlightAxisUnittest, TestAxisIndices)
{
    // In various places Ninjaflight assumes equality between the flight dynamics indices,
    // and angle indices, the RC controls indices, and the PID indices.
    // This test asserts those equalities.

    // check the FD indices have the correct absolute values
    EXPECT_EQ(0, FD_ROLL);
    EXPECT_EQ(1, FD_PITCH);
    EXPECT_EQ(2, FD_YAW);
    EXPECT_EQ(3, FD_INDEX_COUNT);

    // check the AI indices match the FD indices
    EXPECT_EQ(FD_ROLL, AI_ROLL);
    EXPECT_EQ(FD_PITCH, AI_PITCH);
    EXPECT_EQ(2, ANGLE_INDEX_COUNT);

    // check the RC indices match the FD indices
    EXPECT_EQ(FD_ROLL, ROLL);
    EXPECT_EQ(FD_PITCH, PITCH);
    EXPECT_EQ(FD_YAW, YAW);
    EXPECT_EQ(3, THROTTLE); // throttle is sometimes used the fourth item in a zero based array

    // check the PID indices match the FD indices
    EXPECT_EQ(FD_ROLL, PIDROLL);
    EXPECT_EQ(FD_PITCH, PIDPITCH);
    EXPECT_EQ(FD_YAW, PIDYAW);
}

TEST(MixerPresets, TestPresets){
	// check that our indices are correct so that they can be used interchangably
	EXPECT_EQ(MIXER_OUTPUT_MOTORS, 0);
	//EXPECT_EQ(MIXER_OUTPUT_SERVOS, MIXER_OUTPUT_MOTORS + MAX_SUPPORTED_MOTORS);
	//EXPECT_EQ(MIXER_OUTPUT_COUNT, MAX_SUPPORTED_MOTORS + MAX_SUPPORTED_SERVOS);
}

static int testedModes = 0;
class MixerBasicTest : public ::testing::Test {
protected:
    virtual void SetUp() {
		_init_mixer_defaults(MIXER_QUADX);
		mock_system_reset();
		config_reset(&config);
    }

	void _init_mixer_defaults(mixer_mode_t mode){
		config.mixer.mixerMode = mode;
		for(int c = 0; c < 8; c++){
			struct servo_config *conf = &config_get_profile_rw(&config)->servos.servoConf[c];
			conf->middle = 1500;
			conf->rate = 100;
			conf->min = 1000;
			conf->max = 2000;
		}
		mixer_init(&mixer, &config, &mock_syscalls()->pwm);
		mixer_enable_armed(&mixer, true);
	}

	virtual void TearDown(){
		// check that mixer did not write any out of range values into the pwm outputs while running tests
		EXPECT_EQ(0, mock_pwm_errors);
	}

protected:
	struct mixer mixer;
	struct config config;
};

/**
 * @page MIXER
 * @ingroup MIXER
 *
 * - When mixer is started and not armed it will output G4 inputs as
 * passthrough to the motors and will output midrc values on servos. When
 * armed, mixer shall output minimum value that was set by
 * mixer_set_throttle_range.
 **/
TEST_F(MixerBasicTest, TestMixerArmed){
	testedModes = 0;
	// test mixer armed in normal mode
	config.rx.mincheck = 1010;
	config.pwm_out.mincommand = 1010;
	config.pwm_out.minthrottle = 1020;
	config.pwm_out.maxthrottle = 2000;

	_init_mixer_defaults(MIXER_QUADX);

	resetRX();

	EXPECT_EQ(4, mixer_get_motor_count(&mixer));
	EXPECT_EQ(0, mixer_get_servo_count(&mixer));

	mixer_input_command(&mixer, MIXER_INPUT_G0_ROLL, 0);
	mixer_input_command(&mixer, MIXER_INPUT_G0_PITCH, 0);
	mixer_input_command(&mixer, MIXER_INPUT_G0_YAW, 0);

	// udpate the mixer once
	mixer_enable_armed(&mixer, false);
	mixer_update(&mixer);

	// check that all motor outputs have been set to mincommand
	for(int c = 0; c < mixer_get_motor_count(&mixer); c++){
		EXPECT_EQ(config.pwm_out.mincommand, mock_motor_pwm[c]);
	}
	// check that servo outputs have been centered
	for(int c = 0; c < mixer_get_servo_count(&mixer); c++){
		EXPECT_EQ(config.rx.midrc, mock_servo_pwm[c]);
	}

	// try arming (should change them to minthrottle)
	mixer_enable_armed(&mixer, true);
	mixer_update(&mixer);

	// check that mixer outputs have been changed to minthrottle
	for(int c = 0; c < mixer_get_motor_count(&mixer); c++){
		EXPECT_EQ(config.pwm_out.minthrottle, mock_motor_pwm[c]);
	}

	// enable motor stop
	//mixer_enable_motor_stop(&mixer, true);
	mixer_set_throttle_range(&mixer, 1500, config.pwm_out.mincommand, config.pwm_out.maxthrottle);
	mixer_update(&mixer);

	// verify that mixer outputs have been set to mincommand since motorstop should set them to that
	for(int c = 0; c < mixer_get_motor_count(&mixer); c++){
		EXPECT_EQ(config.pwm_out.mincommand, mock_motor_pwm[c]);
	}
}

/**
 * @page MIXER
 * @ingroup MIXER
 *
 * - 3d mode is set in mixer using mixer_set_throttle_range(). TODO: this test
 * needs to be improved.
 */
TEST_F(MixerBasicTest, Test3dThrottleRange){
	_init_mixer_defaults(MIXER_QUADX);

	mixer_set_throttle_range(&mixer, 1250, 1050, 1450);

	// in 3d mode all inputs will be halved
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, -150);
	mixer_update(&mixer);

	// check that outputs match midthrottle - throttle input
	EXPECT_EQ(1100, mock_motor_pwm[0]);
	EXPECT_EQ(1100, mock_motor_pwm[1]);
	EXPECT_EQ(1100, mock_motor_pwm[2]);
	EXPECT_EQ(1100, mock_motor_pwm[3]);

	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, 0);
	mixer_update(&mixer);

	EXPECT_EQ(1250, mock_motor_pwm[0]);
	EXPECT_EQ(1250, mock_motor_pwm[1]);
	EXPECT_EQ(1250, mock_motor_pwm[2]);
	EXPECT_EQ(1250, mock_motor_pwm[3]);
}

/**
 * @page MIXER
 * @ingroup MIXER
 *
 * - When mixer is disarmed it should pass MIXER_INPUT_G4_Mx inputs to the
 * motors and should keep servos at midpoint.
 */
TEST_F(MixerBasicTest, TestMotorPassthroughWhenDisarmed){
	config.pwm_out.mincommand = 1000;

	_init_mixer_defaults(MIXER_QUADX);

	// this is used in the passthrough so must be set to known values
	mixer_set_throttle_range(&default_mixer, 1500, 1000, 2000);

	// output a command to each motor
	// all inputs are 0 based and have range of -500 to 500
	mixer_input_command(&mixer, MIXER_INPUT_G4_M1, 1);
	mixer_input_command(&mixer, MIXER_INPUT_G4_M2, 2);
	mixer_input_command(&mixer, MIXER_INPUT_G4_M3, 3);
	mixer_input_command(&mixer, MIXER_INPUT_G4_M4, 4);
	mixer_input_command(&mixer, MIXER_INPUT_G4_M5, 5);
	mixer_input_command(&mixer, MIXER_INPUT_G4_M6, 6);
	mixer_input_command(&mixer, MIXER_INPUT_G4_M7, 7);
	mixer_input_command(&mixer, MIXER_INPUT_G4_M8, 8);

	mixer_enable_armed(&mixer, false);
	mixer_update(&mixer);

    // then
    for (uint8_t i = 0; i < MIXER_MAX_MOTORS; i++) {
        EXPECT_EQ(1501 + i, mock_motor_pwm[i]);
    }
	// check that servo outputs have been centered
	for(int c = 0; c < mixer_get_servo_count(&mixer); c++){
		EXPECT_EQ(config.rx.midrc, mock_servo_pwm[c]);
	}
}

/**
 * @page MIXER
 * @ingroup MIXER
 *
 * - Inputs in group G3 that correspond to AUX channels should always be
 * automatically forwarded to servos that are not controlled by the mixer
 * (starting with the next servo after the last one that is controlled by the
 * mixer rules). TODO: make sure that this output is mixed according to servo
 * configuration rules.
 */
TEST_F(MixerBasicTest, TestForwardAuxChannelsToServosWithNoServos){
	// center all servos
	mixer_input_command(&mixer, MIXER_INPUT_G3_RC_AUX1, 0);
	mixer_input_command(&mixer, MIXER_INPUT_G3_RC_AUX2, 0);
	mixer_input_command(&mixer, MIXER_INPUT_G3_RC_AUX3, 0);

	mixer_update(&mixer);

    // then
	EXPECT_EQ(1500, mock_servo_pwm[0]);
	EXPECT_EQ(1500, mock_servo_pwm[1]);
	EXPECT_EQ(1500, mock_servo_pwm[2]);

	// try setting the input to something else
	mixer_input_command(&mixer, MIXER_INPUT_G3_RC_AUX1, 100);
	mixer_input_command(&mixer, MIXER_INPUT_G3_RC_AUX2, -100);
	mixer_input_command(&mixer, MIXER_INPUT_G3_RC_AUX3, -1000);

	mixer_update(&mixer);

	EXPECT_EQ(1600, mock_servo_pwm[0]);
	EXPECT_EQ(1400, mock_servo_pwm[1]);
	EXPECT_EQ(config.pwm_out.mincommand, mock_servo_pwm[2]);
}

/**
 * @page MIXER
 * @ingroup MIXER
 *
 * - Mixer scales motor output such that differential thrust is maintained as
 * much as possible. If some motors are above maximum output range, mixer moves
 * all motors down such that the difference between motor with least thrust and
 * motor with most thrust is maintained. If both top and bottom limits are
 * exceeded then all motors are scaled to fit in the allowed output range.
 */
TEST_F(MixerBasicTest, TestMixerExtremes){
	// set up some config defaults
	config.rx.mincheck = 1010;
	config.rx.midrc = 1500;
	config.pwm_out.mincommand = 1000;
	config.pwm_out.minthrottle = 1100;
	config.pwm_out.maxthrottle = 1850;

	_init_mixer_defaults(MIXER_QUADX);

	// we have 4 motors and 0 managed servos
	EXPECT_EQ(4, mixer_get_motor_count(&mixer));
	EXPECT_EQ(0, mixer_get_servo_count(&mixer));

	mixer_enable_armed(&mixer, true);

	// test throttle response without giving any commands
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, -500);
	mixer_update(&mixer);

	// expect minthrottle on motors
	EXPECT_EQ(config.pwm_out.minthrottle, mock_motor_pwm[0]);
	EXPECT_EQ(config.pwm_out.minthrottle, mock_motor_pwm[1]);
	EXPECT_EQ(config.pwm_out.minthrottle, mock_motor_pwm[2]);
	EXPECT_EQ(config.pwm_out.minthrottle, mock_motor_pwm[3]);

	// input some throttle
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, -200);
	mixer_update(&mixer);

	// we expect all motors to be at 1100 throttle range
	EXPECT_EQ(1300, mock_motor_pwm[0]);
	EXPECT_EQ(1300, mock_motor_pwm[1]);
	EXPECT_EQ(1300, mock_motor_pwm[2]);
	EXPECT_EQ(1300, mock_motor_pwm[3]);

	// try pitching forward 100 (1/5 of half throttle range)
	mixer_input_command(&mixer, MIXER_INPUT_G0_PITCH, 100);
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, 0);
	mixer_update(&mixer);

	// we expect differential thrust for quadx to be increased by 100 units on back motors and decreased by the same ammount on front motors
	EXPECT_EQ(1600, mock_motor_pwm[0]);
	EXPECT_EQ(1400, mock_motor_pwm[1]);
	EXPECT_EQ(1600, mock_motor_pwm[2]);
	EXPECT_EQ(1400, mock_motor_pwm[3]);

	// try pitching forward 100 with maxthrottle
	mixer_input_command(&mixer, MIXER_INPUT_G0_PITCH, 100);
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, 500);
	mixer_update(&mixer);

	// we expect the response to be moved down in the throttle range to fit into maxthrottle
	EXPECT_EQ(1850, mock_motor_pwm[0]);
	EXPECT_EQ(1650, mock_motor_pwm[1]);
	EXPECT_EQ(1850, mock_motor_pwm[2]);
	EXPECT_EQ(1650, mock_motor_pwm[3]);

	// now test the same thing but with minimum throttle
	mixer_input_command(&mixer, MIXER_INPUT_G0_PITCH, 100);
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, -500);
	mixer_update(&mixer);

	// we expect the response to be moved down in the throttle range to fit into maxthrottle
	EXPECT_EQ(1300, mock_motor_pwm[0]);
	EXPECT_EQ(1100, mock_motor_pwm[1]);
	EXPECT_EQ(1300, mock_motor_pwm[2]);
	EXPECT_EQ(1100, mock_motor_pwm[3]);


	// attempt to provide full pitch and check that we get motor limit reached flag set
	mixer_input_command(&mixer, MIXER_INPUT_G0_PITCH, 500);
	mixer_update(&mixer);
}

/**
 * @page MIXER
 * @ingroup MIXER
 *
 * - Motors that are unused by the mixer are always held at *mincommand*
 * setting as specified in the mixer configuration. This only applies to motor
 * outputs (not servos).
 */
TEST_F(MixerBasicTest, TestMixerUnusedMotorsAtMin){
	// set up some config defaults
	config.rx.mincheck = 1010;
	config.rx.midrc = 1500;
	config.pwm_out.mincommand = 1010;
	config.pwm_out.minthrottle = 1050;
	config.pwm_out.maxthrottle = 1850;

	_init_mixer_defaults(MIXER_QUADX);

	mixer_update(&mixer);

	for(int c = mixer_get_motor_count(&mixer); c < MIXER_MAX_MOTORS; c++){
		EXPECT_EQ(config.pwm_out.mincommand, mock_motor_pwm[c]);
	}
}

TEST_F(MixerBasicTest, TestMixerModeQuadX){
	// set up some config defaults
	config.rx.mincheck = 1010;
	config.rx.midrc = 1500;
	config.pwm_out.mincommand = 1000;
	config.pwm_out.minthrottle = 1050;
	config.pwm_out.maxthrottle = 1850;

	_init_mixer_defaults(MIXER_QUADX);

	// we have 4 motors and 0 managed servos
	EXPECT_EQ(4, mixer_get_motor_count(&mixer));
	EXPECT_EQ(0, mixer_get_servo_count(&mixer));

	mixer_enable_armed(&mixer, true);

	// test throttle response without giving any commands
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, -500);
	mixer_update(&mixer);

	// motor stop is off so we expect minthrottle
	EXPECT_EQ(config.pwm_out.minthrottle, mock_motor_pwm[0]);
	EXPECT_EQ(config.pwm_out.minthrottle, mock_motor_pwm[1]);
	EXPECT_EQ(config.pwm_out.minthrottle, mock_motor_pwm[2]);
	EXPECT_EQ(config.pwm_out.minthrottle, mock_motor_pwm[3]);

	// input some throttle
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, -400);
	mixer_update(&mixer);

	// we expect all motors to be at 1100 throttle range
	EXPECT_EQ(1100, mock_motor_pwm[0]);
	EXPECT_EQ(1100, mock_motor_pwm[1]);
	EXPECT_EQ(1100, mock_motor_pwm[2]);
	EXPECT_EQ(1100, mock_motor_pwm[3]);

	// try pitching forward 100 (1/5 of half throttle range)
	mixer_input_command(&mixer, MIXER_INPUT_G0_PITCH, 100);
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, 0);
	mixer_update(&mixer);

	// we expect differential thrust for quadx to be increased by 100 units on back motors and decreased by the same ammount on front motors
	EXPECT_EQ(1600, mock_motor_pwm[0]);
	EXPECT_EQ(1400, mock_motor_pwm[1]);
	EXPECT_EQ(1600, mock_motor_pwm[2]);
	EXPECT_EQ(1400, mock_motor_pwm[3]);

	// attempt to provide full pitch and check that we get motor limit reached flag set
	mixer_input_command(&mixer, MIXER_INPUT_G0_PITCH, 500);
	mixer_update(&mixer);

	EXPECT_EQ(true, mixer_motor_limit_reached(&mixer));

	// attempt to decrease command and test that motor limit is not reached
	mixer_input_command(&mixer, MIXER_INPUT_G0_PITCH, 0);
	mixer_update(&mixer);

	EXPECT_EQ(false, mixer_motor_limit_reached(&mixer));

	// expect all motors to be at throttle value
	EXPECT_EQ(1500, mock_motor_pwm[0]);
	EXPECT_EQ(1500, mock_motor_pwm[1]);
	EXPECT_EQ(1500, mock_motor_pwm[2]);
	EXPECT_EQ(1500, mock_motor_pwm[3]);

	testedModes++;
}

TEST_F(MixerBasicTest, TestMixerModeAirplane){
	// set up some config defaults
	config.rx.mincheck = 1010;
	config.rx.midrc = 1500;
	config.pwm_out.mincommand = 1000;
	config.pwm_out.minthrottle = 1050;
	config.pwm_out.maxthrottle = 1850;

	_init_mixer_defaults(MIXER_AIRPLANE);

	// we have 4 motors and 0 managed servos
	EXPECT_EQ(1, mixer_get_motor_count(&mixer));
	EXPECT_EQ(5, mixer_get_servo_count(&mixer));

	mixer_enable_armed(&mixer, true);

	// test throttle response without giving any commands
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, -100);
	mixer_update(&mixer);

	// motor stop is off so we expect minthrottle
	EXPECT_EQ(1400, mock_motor_pwm[0]);
	EXPECT_EQ(1500, mock_servo_pwm[0]);
	EXPECT_EQ(1500, mock_servo_pwm[1]);
	EXPECT_EQ(1500, mock_servo_pwm[2]);
	EXPECT_EQ(1500, mock_servo_pwm[3]);
	// make sure the throttle servo works too
	EXPECT_EQ(1400, mock_servo_pwm[4]);

	// try pitching forward 100
	mixer_input_command(&mixer, MIXER_INPUT_G0_PITCH, 100);
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, 100);
	mixer_update(&mixer);

	// expect elevator to have moved
	EXPECT_EQ(1500, mock_servo_pwm[0]);
	EXPECT_EQ(1500, mock_servo_pwm[1]);
	EXPECT_EQ(1500, mock_servo_pwm[2]);
	EXPECT_EQ(1600, mock_servo_pwm[3]);
	EXPECT_EQ(1600, mock_servo_pwm[4]);

	// attempt to provide full pitch and check that we get motor limit reached flag set
	mixer_input_command(&mixer, MIXER_INPUT_G0_ROLL, 500);
	mixer_update(&mixer);

	// doesn't really matter but we just test that we did not reach motor limit.
	EXPECT_EQ(false, mixer_motor_limit_reached(&mixer));

	// expect aelerons to have moved
	EXPECT_EQ(2000, mock_servo_pwm[0]);
	EXPECT_EQ(2000, mock_servo_pwm[1]);
	EXPECT_EQ(1500, mock_servo_pwm[2]);
	EXPECT_EQ(1600, mock_servo_pwm[3]);
	EXPECT_EQ(1600, mock_servo_pwm[4]);

	testedModes++;
}

TEST_F(MixerBasicTest, TestQuadMotors)
{
	config.pwm_out.mincommand = 1000;
	config.pwm_out.minthrottle = 1000;
	config.pwm_out.maxthrottle = 2000;

    _init_mixer_defaults(MIXER_QUADX);

    // when
    mixer_update(&mixer);

    // then
    EXPECT_EQ(4, mixer_get_motor_count(&mixer));

    EXPECT_EQ(TEST_MIN_COMMAND, mock_motor_pwm[0]);
    EXPECT_EQ(TEST_MIN_COMMAND, mock_motor_pwm[1]);
    EXPECT_EQ(TEST_MIN_COMMAND, mock_motor_pwm[2]);
    EXPECT_EQ(TEST_MIN_COMMAND, mock_motor_pwm[3]);
}

/**
 * @page MIXER
 * @ingroup MIXER
 *
 * - Mixer has basic tolerance against invalid configuration values. Values are
 * constrained in some places. Do keep configuration valid for best result.
 */
TEST_F(MixerBasicTest, TestInvalidConfig){
	// this test will just fill configs with random data and we will see if mixer performs well
	memset(&config, 0xff, sizeof(struct config));

	// use quadx
	config.mixer.mixerMode = MIXER_QUADX;

	// need to set this 
	config.pwm_out.mincommand = 1000;

	mixer_init(&mixer, &config, &mock_syscalls()->pwm);
	mixer_enable_armed(&mixer, true);

    // when
    mixer_update(&mixer);

    // then
    EXPECT_EQ(4, mixer_get_motor_count(&mixer));

    EXPECT_EQ(TEST_MIN_COMMAND, mock_motor_pwm[0]);
    EXPECT_EQ(TEST_MIN_COMMAND, mock_motor_pwm[1]);
    EXPECT_EQ(TEST_MIN_COMMAND, mock_motor_pwm[2]);
    EXPECT_EQ(TEST_MIN_COMMAND, mock_motor_pwm[3]);
}

TEST_F(MixerBasicTest, TestAllModesTested){
	//EXPECT_EQ(MIXER_MODE_COUNT, testedModes);
}


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

    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"
	
	#include "config/gimbal.h"

    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "drivers/pwm_mapping.h"
    #include "drivers/gyro_sync.h"

    #include "sensors/acceleration.h"

    #include "rx/rx.h"
    #include "flight/anglerate.h"
    #include "flight/mixer.h"

    #include "io/rc_controls.h"

    #include "config/config.h"

    //void mixerUsePWMIOConfiguration(struct mixer *self, pwmIOConfiguration_t *pwmIOConfiguration);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

struct mixer default_mixer; 

// input
#define TEST_RC_MID 1500

// output
#define TEST_MIN_COMMAND 1000
#define TEST_SERVO_MID 1500

typedef struct motor_s {
    uint16_t value;
} motor_t;

typedef struct servo_s {
    uint16_t value;
} servo_t;

motor_t motors[MAX_SUPPORTED_MOTORS];
servo_t servos[MAX_SUPPORTED_SERVOS];

uint8_t lastOneShotUpdateMotorCount;

uint32_t testFeatureMask = 0;

int updatedServoCount;
int updatedMotorCount;

// STUBS

extern "C" {
rxRuntimeConfig_t rxRuntimeConfig;

struct pid_controller_output pid_output; 
int16_t rcCommand[4];

uint32_t rcModeActivationMask;
int16_t debug[DEBUG16_VALUE_COUNT];

uint8_t stateFlags;
uint16_t flightModeFlags;
uint8_t armingFlags;

uint32_t targetLooptime;

float applyBiQuadFilter(float sample, biquad_t *state) {UNUSED(state);return sample;}
void BiQuadNewLpf(float filterCutFreq, biquad_t *newState, uint32_t refreshRate) {UNUSED(filterCutFreq);UNUSED(newState);UNUSED(refreshRate);}


bool feature(uint32_t mask) {
    return (mask & testFeatureMask);
}

void pwmWriteMotor(uint8_t index, uint16_t value) {
    motors[index].value = value;
    updatedMotorCount++;
}

void pwmShutdownPulsesForAllMotors(uint8_t motorCount)
{
    uint8_t index;

    for(index = 0; index < motorCount; index++){
        motors[index].value = 0;
    }
}

void pwmCompleteOneshotMotorUpdate(uint8_t motorCount) {
    lastOneShotUpdateMotorCount = motorCount;
}

void pwmWriteServo(uint8_t index, uint16_t value) {
    // FIXME logic in test, mimic's production code.
    // Perhaps the solution is to remove the logic from the production code version and assume that
    // anything calling calling pwmWriteServo always uses a valid index?
    // See MAX_SERVOS in pwm_output (driver) and MAX_SUPPORTED_SERVOS (flight)
    if (index < MAX_SERVOS) {
        servos[index].value = value;
    }
    updatedServoCount++;
}

bool rcModeIsActive(boxId_e modeId) { return rcModeActivationMask & (1 << modeId); }
}

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
        memset(&servos, 0, sizeof(servos));
		_init_mixer_defaults(MIXER_QUADX);
		mock_system_reset();
    }

	void _init_mixer_defaults(mixer_mode_t mode){
		mixerConfig()->mixerMode = mode;
		for(int c = 0; c < 8; c++){
			struct servo_config *conf = &servoProfile()->servoConf[c];
			conf->middle = 1500;
			conf->rate = 100;
			conf->min = 1000;
			conf->max = 2000;
		}
		mixer_init(&mixer,
			mixerConfig(),
			motor3DConfig(),
			motorAndServoConfig(),
			rxConfig(),
			rcControlsConfig(),
			servoProfile()->servoConf,
			&mock_syscalls()->pwm,
			customMotorMixer(0), MAX_SUPPORTED_MOTORS);
		mixer_enable_armed(&mixer, true);
	}

	virtual void TearDown(){
		// check that mixer did not write any out of range values into the pwm outputs while running tests
		EXPECT_EQ(0, mock_pwm_errors);
	}

protected:
	struct mixer mixer;
};

TEST_F(MixerBasicTest, TestMixerArmed){
	testedModes = 0;
	// test mixer armed in normal mode
	rxConfig()->mincheck = 1010;
	motorAndServoConfig()->mincommand = 1010;
	motorAndServoConfig()->minthrottle = 1020;
	motorAndServoConfig()->maxthrottle = 2000;

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
		EXPECT_EQ(motorAndServoConfig()->mincommand, mock_motor_pwm[c]);
	}

	// try arming (should change them to minthrottle)
	mixer_enable_armed(&mixer, true);
	mixer_update(&mixer);

	// check that mixer outputs have been changed to minthrottle
	for(int c = 0; c < mixer_get_motor_count(&mixer); c++){
		EXPECT_EQ(motorAndServoConfig()->minthrottle, mock_motor_pwm[c]);
	}

	// enable motor stop
	//mixer_enable_motor_stop(&mixer, true);
	mixer_set_throttle_range(&mixer, 1500, motorAndServoConfig()->mincommand, motorAndServoConfig()->maxthrottle);
	mixer_update(&mixer);

	// verify that mixer outputs have been set to mincommand since motorstop should set them to that
	for(int c = 0; c < mixer_get_motor_count(&mixer); c++){
		EXPECT_EQ(motorAndServoConfig()->mincommand, mock_motor_pwm[c]);
	}
}

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

TEST_F(MixerBasicTest, TestMotorPassthroughWhenDisarmed){
	motorAndServoConfig()->mincommand = 1000;

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
}


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
	EXPECT_EQ(motorAndServoConfig()->mincommand, mock_servo_pwm[2]);
}

TEST_F(MixerBasicTest, TestMixerExtremes){
	// set up some config defaults
	rxConfig()->mincheck = 1010;
	rxConfig()->midrc = 1500;
	motorAndServoConfig()->mincommand = 1000;
	motorAndServoConfig()->minthrottle = 1100;
	motorAndServoConfig()->maxthrottle = 1850;

	_init_mixer_defaults(MIXER_QUADX);

	// we have 4 motors and 0 managed servos
	EXPECT_EQ(4, mixer_get_motor_count(&mixer));
	EXPECT_EQ(0, mixer_get_servo_count(&mixer));

	mixer_enable_armed(&mixer, true);

	// test throttle response without giving any commands
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, -500);
	mixer_update(&mixer);

	// expect minthrottle on motors
	EXPECT_EQ(motorAndServoConfig()->minthrottle, mock_motor_pwm[0]);
	EXPECT_EQ(motorAndServoConfig()->minthrottle, mock_motor_pwm[1]);
	EXPECT_EQ(motorAndServoConfig()->minthrottle, mock_motor_pwm[2]);
	EXPECT_EQ(motorAndServoConfig()->minthrottle, mock_motor_pwm[3]);

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

TEST_F(MixerBasicTest, TestMixerUnusedMotorsAtMin){
	// set up some config defaults
	rxConfig()->mincheck = 1010;
	rxConfig()->midrc = 1500;
	motorAndServoConfig()->mincommand = 1010;
	motorAndServoConfig()->minthrottle = 1050;
	motorAndServoConfig()->maxthrottle = 1850;

	_init_mixer_defaults(MIXER_QUADX);

	mixer_update(&mixer);

	for(int c = mixer_get_motor_count(&mixer); c < MIXER_MAX_MOTORS; c++){
		EXPECT_EQ(motorAndServoConfig()->mincommand, mock_motor_pwm[c]);
	}
}

TEST_F(MixerBasicTest, TestMixerModeQuadX){
	// set up some config defaults
	rxConfig()->mincheck = 1010;
	rxConfig()->midrc = 1500;
	motorAndServoConfig()->mincommand = 1000;
	motorAndServoConfig()->minthrottle = 1050;
	motorAndServoConfig()->maxthrottle = 1850;

	_init_mixer_defaults(MIXER_QUADX);

	// we have 4 motors and 0 managed servos
	EXPECT_EQ(4, mixer_get_motor_count(&mixer));
	EXPECT_EQ(0, mixer_get_servo_count(&mixer));

	mixer_enable_armed(&mixer, true);

	// test throttle response without giving any commands
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, -500);
	mixer_update(&mixer);

	// motor stop is off so we expect minthrottle
	EXPECT_EQ(motorAndServoConfig()->minthrottle, mock_motor_pwm[0]);
	EXPECT_EQ(motorAndServoConfig()->minthrottle, mock_motor_pwm[1]);
	EXPECT_EQ(motorAndServoConfig()->minthrottle, mock_motor_pwm[2]);
	EXPECT_EQ(motorAndServoConfig()->minthrottle, mock_motor_pwm[3]);

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
	rxConfig()->mincheck = 1010;
	rxConfig()->midrc = 1500;
	motorAndServoConfig()->mincommand = 1000;
	motorAndServoConfig()->minthrottle = 1050;
	motorAndServoConfig()->maxthrottle = 1850;

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

TEST_F(MixerBasicTest, TestMixerLoadSave){
	// set up some config defaults
	rxConfig()->mincheck = 1010;
	rxConfig()->midrc = 1500;
	motorAndServoConfig()->mincommand = 1000;
	motorAndServoConfig()->minthrottle = 1050;
	motorAndServoConfig()->maxthrottle = 1850;

	// test QUAD, TRICOPTER and AIRPLANE to get maximal code coverage!
	struct test_def {
		const char *name;
		int motors;
		int servos;
		int rules;
		mixer_mode_t type;
	} tests[] = {
		{ .name = "AIRPLANE", .motors = 1, .servos = 5, .rules = 6, .type = MIXER_AIRPLANE },
		{ .name = "QUADX", .motors = 4, .servos = 0, .rules = 16, .type = MIXER_QUADX },
		{ .name = "TRI", .motors = 3, .servos = 1, .rules = 9, .type = MIXER_TRI }
	};
	for(int t = 0; t < 3; t++){
		struct test_def *cur = &tests[t];

		printf("testing %s\n", cur->name);
		_init_mixer_defaults(cur->type);

		EXPECT_EQ(cur->motors, mixer_get_motor_count(&mixer));
		EXPECT_EQ(cur->servos, mixer_get_servo_count(&mixer));

		struct motor_mixer motors[8], motors2[8];
		struct servo_mixer servos[8], servos2[8];

		memset(motors, 0, sizeof(motors));
		memset(motors2, 0, sizeof(motors2));
		memset(servos, 0, sizeof(servos));
		memset(servos2, 0, sizeof(servos2));

		struct mixer_rule_def rules[sizeof(mixer.active_rules) / sizeof(struct mixer_rule_def)];
		EXPECT_EQ(sizeof(rules), sizeof(mixer.active_rules));
		EXPECT_EQ(cur->rules, mixer.ruleCount);

		memcpy(rules, mixer.active_rules, sizeof(mixer.active_rules));

		EXPECT_EQ(mixer_save_motor_mixer(&mixer, motors), cur->motors);
		EXPECT_EQ(mixer_save_servo_mixer(&mixer, servos), cur->servos);
		
		mixer_clear_rules(&mixer);
		
		mixer_load_motor_mixer(&mixer, motors);
		mixer_load_servo_mixer(&mixer, servos);
		
		mixer_load_motor_mixer(&mixer, motors);
		mixer_load_servo_mixer(&mixer, servos);

		EXPECT_EQ(cur->rules, mixer.ruleCount);
		EXPECT_EQ(memcmp(rules, mixer.active_rules, sizeof(struct mixer_rule_def) * mixer.ruleCount), 0);

		// try saving again and compare
		EXPECT_EQ(mixer_save_motor_mixer(&mixer, motors2), cur->motors);
		EXPECT_EQ(mixer_save_servo_mixer(&mixer, servos2), cur->servos);
		
		for(int c = 0; c < 8; c++){
			if(motors[c].throttle < 1e-6 || motors2[c].throttle < 1e-6) break;
			EXPECT_FLOAT_EQ(motors[c].roll, motors2[c].roll);
			EXPECT_FLOAT_EQ(motors[c].pitch, motors2[c].pitch);
			EXPECT_FLOAT_EQ(motors[c].yaw, motors2[c].yaw);
			EXPECT_FLOAT_EQ(motors[c].throttle, motors2[c].throttle);
		}
		for(int c = 0; c < mixer_get_servo_count(&mixer); c++){
			printf("servo: from:%d to:%d rate:%d\n", servos[c].inputSource, servos[c].targetChannel, servos[c].rate);
			EXPECT_EQ(servos[c].inputSource, servos2[c].inputSource);
			EXPECT_EQ(servos[c].targetChannel, servos2[c].targetChannel);
			EXPECT_EQ(servos[c].rate, servos2[c].rate);
		}
	}
}

TEST_F(MixerBasicTest, TestQuadMotors)
{
	motorAndServoConfig()->mincommand = 1000;
	motorAndServoConfig()->minthrottle = 1000;
	motorAndServoConfig()->maxthrottle = 2000;

    _init_mixer_defaults(MIXER_QUADX);
    // and
    memset(rcCommand, 0, sizeof(rcCommand));

    // and
    memset(&pid_output, 0, sizeof(pid_output));
	pid_output.axis[FD_YAW] = 0; 

    // when
    mixer_update(&mixer);

    // then
    EXPECT_EQ(4, mixer_get_motor_count(&mixer));

    EXPECT_EQ(TEST_MIN_COMMAND, mock_motor_pwm[0]);
    EXPECT_EQ(TEST_MIN_COMMAND, mock_motor_pwm[1]);
    EXPECT_EQ(TEST_MIN_COMMAND, mock_motor_pwm[2]);
    EXPECT_EQ(TEST_MIN_COMMAND, mock_motor_pwm[3]);
}

TEST_F(MixerBasicTest, TestInvalidConfig){
	// this test will just fill configs with random data and we will see if mixer performs well
	memset(mixerConfig(), 0xff, sizeof(struct mixer_config));
	memset(motor3DConfig(), 0xff, sizeof(struct motor_3d_config));
	memset(motorAndServoConfig(), 0xff, sizeof(motorAndServoConfig_t));
	memset(rxConfig(), 0xff, sizeof(rxConfig_t));
	memset(servoProfile(), 0xff, sizeof(struct servo_profile));

	// use quadx
	mixerConfig()->mixerMode = MIXER_QUADX;

	// need to set this 
	motorAndServoConfig()->mincommand = 1000;

	mixer_init(&mixer,
			mixerConfig(),
			motor3DConfig(),
			motorAndServoConfig(),
			rxConfig(),
			rcControlsConfig(),
			servoProfile()->servoConf,
			&mock_syscalls()->pwm,
			customMotorMixer(0), MAX_SUPPORTED_MOTORS);
	mixer_enable_armed(&mixer, true);

    // and
    memset(rcCommand, 0, sizeof(rcCommand));

    // and
    memset(&pid_output, 0, sizeof(pid_output));
	pid_output.axis[FD_YAW] = 0; 

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

class BasicMixerIntegrationTest : public ::testing::Test {
protected:
    struct servo_config servoConf[MAX_SUPPORTED_SERVOS];
    gimbalConfig_t gimbalConfig = {
        .mode = GIMBAL_MODE_NORMAL
    };

    virtual void SetUp() {
		mock_system_reset();

        updatedServoCount = 0;
        updatedMotorCount = 0;
    }

    virtual void withDefaultmotorAndServoConfiguration(void) {
        motorAndServoConfig()->mincommand = TEST_MIN_COMMAND;
    }

    virtual void withDefaultRxConfig(void) {
        rxConfig()->midrc = 1500;
    }

    virtual void configureMixer(uint8_t mixerMode) {
        mixerConfig()->mixerMode = mixerMode;
    }
};

class CustomMixerIntegrationTest : public BasicMixerIntegrationTest {
protected:

    virtual void SetUp() {
		mock_system_reset();

        BasicMixerIntegrationTest::SetUp();

        memset(&servoConf, 0, sizeof(servoConf));
        for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servoConf[i].min = DEFAULT_SERVO_MIN;
            servoConf[i].max = DEFAULT_SERVO_MAX;
            servoConf[i].middle = DEFAULT_SERVO_MIDDLE;
            servoConf[i].rate = 100;
            servoConf[i].forwardFromChannel = CHANNEL_FORWARDING_DISABLED;
        }

        withDefaultmotorAndServoConfiguration();
        withDefaultRxConfig();

        configureMixer(MIXER_QUADX);

        memset(*customMotorMixer_arr(), 0, sizeof(*customMotorMixer_arr()));
        memset(*customServoMixer_arr(), 0, sizeof(*customServoMixer_arr()));
    }
};

TEST_F(CustomMixerIntegrationTest, TestCustomMixer)
{
	struct mixer mixer;
    // given
    enum {
        EXPECTED_SERVOS_TO_MIX_COUNT = 6,
        EXPECTED_MOTORS_TO_MIX_COUNT = 2
    };

    struct servo_mixer testServoMixer[EXPECTED_SERVOS_TO_MIX_COUNT] = {
        { MIXER_OUTPUT_S1, MIXER_INPUT_G0_PITCH, 100, 0, 0, 100, 0 },
        { MIXER_OUTPUT_S2, MIXER_INPUT_G0_ROLL,  100, 0, 0, 100, 0 },
        { MIXER_OUTPUT_S3, MIXER_INPUT_G0_ROLL,  100, 0, 0, 100, 0 },
        { MIXER_OUTPUT_S4, MIXER_INPUT_G0_YAW,   100, 0, 0, 100, 0 },
        { MIXER_OUTPUT_S5, MIXER_INPUT_G0_THROTTLE, 100, 0, 0, 100, 0 },
        { MIXER_OUTPUT_S6, MIXER_INPUT_G3_RC_AUX1,  100, 0, 0, 100, 0 }
    };
    memcpy(customServoMixer_arr(), testServoMixer, sizeof(testServoMixer));

    static const struct motor_mixer testMotorMixer[EXPECTED_MOTORS_TO_MIX_COUNT] = {
        { 1.0f,  0.0f,  0.0f, -1.0f },          // LEFT
        { 1.0f,  0.0f,  0.0f,  1.0f }          // RIGHT
    };
    memcpy(customMotorMixer_arr(), testMotorMixer, sizeof(testMotorMixer));

    configureMixer(MIXER_CUSTOM_AIRPLANE);

	motorAndServoConfig()->mincommand = 1000;
	motorAndServoConfig()->minthrottle = 1000;
	motorAndServoConfig()->maxthrottle = 2000;

    mixer_init(&mixer, 
		mixerConfig(),
		motor3DConfig(),
		motorAndServoConfig(),
		rxConfig(),
		rcControlsConfig(),
		servoProfile()->servoConf,
		&mock_syscalls()->pwm,
		customMotorMixer(0), MAX_SUPPORTED_MOTORS);

	for(int c = 0; c < MAX_SUPPORTED_SERVOS; c++){
		servoProfile()->servoConf[c].rate = 100;
		servoProfile()->servoConf[c].max = 2000;
		servoProfile()->servoConf[c].min = 1000;
		servoProfile()->servoConf[c].middle = 1500;
	}

	mixer_load_motor_mixer(&mixer, customMotorMixer(0));
	mixer_load_servo_mixer(&mixer, customServoMixer(0));
    // and
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, -500);
	mixer_input_command(&mixer, MIXER_INPUT_G3_RC_AUX1, 500);

    // when
	mixer_enable_armed(&mixer, true);
    mixer_update(&mixer);

    // then
    EXPECT_EQ(EXPECTED_MOTORS_TO_MIX_COUNT, mixer_get_motor_count(&mixer));
    EXPECT_EQ(EXPECTED_SERVOS_TO_MIX_COUNT, mixer_get_servo_count(&mixer));

    EXPECT_EQ(TEST_MIN_COMMAND, mock_motor_pwm[0]);
    EXPECT_EQ(TEST_MIN_COMMAND, mock_motor_pwm[1]);

    EXPECT_EQ(TEST_SERVO_MID, mock_servo_pwm[0]);
    EXPECT_EQ(TEST_SERVO_MID, mock_servo_pwm[1]);
    EXPECT_EQ(TEST_SERVO_MID, mock_servo_pwm[2]);
    EXPECT_EQ(TEST_SERVO_MID, mock_servo_pwm[3]);
    EXPECT_EQ(1000, mock_servo_pwm[4]); // Throttle
    EXPECT_EQ(2000, mock_servo_pwm[5]); // Flaps
}


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

    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "drivers/pwm_mapping.h"
    #include "drivers/gyro_sync.h"

    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"

    #include "rx/rx.h"
    #include "flight/anglerate.h"
    #include "flight/imu.h"
    #include "flight/mixer.h"

    #include "io/motor_and_servo.h"
    #include "io/gimbal.h"
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
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
// TODO: proper way to do this is to write a mock receiver
int16_t rc_get_channel_value(uint8_t id){ return rcData[id]; }
void rc_set_channel_value(uint8_t id, int16_t value){ rcData[id] = value; }

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
	rcData[ROLL] = 1500;
	rcData[PITCH] = 1500;
	rcData[YAW] = 1500;
	rcData[THROTTLE] = 1000;
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
			customMotorMixer(0), MAX_SUPPORTED_MOTORS);
		mixer_enable_armed(&mixer, true);
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
		EXPECT_EQ(motorAndServoConfig()->mincommand, mixer_get_motor_value(&mixer, c));
	}

	// try arming (should change them to minthrottle)
	mixer_enable_armed(&mixer, true);
	mixer_update(&mixer);

	// check that mixer outputs have been changed to minthrottle
	for(int c = 0; c < mixer_get_motor_count(&mixer); c++){
		EXPECT_EQ(motorAndServoConfig()->minthrottle, mixer_get_motor_value(&mixer, c));
	}

	// enable motor stop
	//mixer_enable_motor_stop(&mixer, true);
	mixer_set_throttle_range(&mixer, 1500, motorAndServoConfig()->mincommand, motorAndServoConfig()->maxthrottle);
	mixer_update(&mixer);

	// verify that mixer outputs have been set to mincommand since motorstop should set them to that
	for(int c = 0; c < mixer_get_motor_count(&mixer); c++){
		EXPECT_EQ(motorAndServoConfig()->mincommand, mixer_get_motor_value(&mixer, c));
	}
}

TEST_F(MixerBasicTest, Test3dThrottleRange){
	_init_mixer_defaults(MIXER_QUADX);

	mixer_set_throttle_range(&mixer, 1250, 1050, 1450);

	// in 3d mode all inputs will be halved
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, -150);
	mixer_update(&mixer);

	// check that outputs match midthrottle - throttle input
	EXPECT_EQ(1100, mixer_get_motor_value(&mixer, 0));
	EXPECT_EQ(1100, mixer_get_motor_value(&mixer, 1));
	EXPECT_EQ(1100, mixer_get_motor_value(&mixer, 2));
	EXPECT_EQ(1100, mixer_get_motor_value(&mixer, 3));

	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, 0);
	mixer_update(&mixer);

	EXPECT_EQ(1250, mixer_get_motor_value(&mixer, 0));
	EXPECT_EQ(1250, mixer_get_motor_value(&mixer, 1));
	EXPECT_EQ(1250, mixer_get_motor_value(&mixer, 2));
	EXPECT_EQ(1250, mixer_get_motor_value(&mixer, 3));


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
        EXPECT_EQ(1501 + i, mixer_get_motor_value(&mixer, i));
    }
}


TEST_F(MixerBasicTest, TestForwardAuxChannelsToServosWithNoServos){
	// center all servos
	mixer_input_command(&mixer, MIXER_INPUT_G3_RC_AUX1, 0);
	mixer_input_command(&mixer, MIXER_INPUT_G3_RC_AUX2, 0);
	mixer_input_command(&mixer, MIXER_INPUT_G3_RC_AUX3, 0);

	mixer_update(&mixer);

    // then
	EXPECT_EQ(1500, mixer_get_servo_value(&mixer, 0));
	EXPECT_EQ(1500, mixer_get_servo_value(&mixer, 1));
	EXPECT_EQ(1500, mixer_get_servo_value(&mixer, 2));

	// try setting the input to something else
	mixer_input_command(&mixer, MIXER_INPUT_G3_RC_AUX1, 100);
	mixer_input_command(&mixer, MIXER_INPUT_G3_RC_AUX2, -100);
	mixer_input_command(&mixer, MIXER_INPUT_G3_RC_AUX3, -1000);

	mixer_update(&mixer);

	EXPECT_EQ(1600, mixer_get_servo_value(&mixer, 0));
	EXPECT_EQ(1400, mixer_get_servo_value(&mixer, 1));
	EXPECT_EQ(motorAndServoConfig()->mincommand, mixer_get_servo_value(&mixer, 2));
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
	EXPECT_EQ(motorAndServoConfig()->minthrottle, mixer_get_motor_value(&mixer, 0));
	EXPECT_EQ(motorAndServoConfig()->minthrottle, mixer_get_motor_value(&mixer, 1));
	EXPECT_EQ(motorAndServoConfig()->minthrottle, mixer_get_motor_value(&mixer, 2));
	EXPECT_EQ(motorAndServoConfig()->minthrottle, mixer_get_motor_value(&mixer, 3));

	// input some throttle
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, -400);
	mixer_update(&mixer);

	// we expect all motors to be at 1100 throttle range
	EXPECT_EQ(1100, mixer_get_motor_value(&mixer, 0));
	EXPECT_EQ(1100, mixer_get_motor_value(&mixer, 1));
	EXPECT_EQ(1100, mixer_get_motor_value(&mixer, 2));
	EXPECT_EQ(1100, mixer_get_motor_value(&mixer, 3));

	// try pitching forward 100 (1/5 of half throttle range)
	mixer_input_command(&mixer, MIXER_INPUT_G0_PITCH, 100);
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, 0);
	mixer_update(&mixer);

	// we expect differential thrust for quadx to be increased by 100 units on back motors and decreased by the same ammount on front motors
	EXPECT_EQ(1600, mixer_get_motor_value(&mixer, 0));
	EXPECT_EQ(1400, mixer_get_motor_value(&mixer, 1));
	EXPECT_EQ(1600, mixer_get_motor_value(&mixer, 2));
	EXPECT_EQ(1400, mixer_get_motor_value(&mixer, 3));

	// attempt to provide full pitch and check that we get motor limit reached flag set
	mixer_input_command(&mixer, MIXER_INPUT_G0_PITCH, 500);
	mixer_update(&mixer);

	EXPECT_EQ(true, mixer_motor_limit_reached(&mixer));

	// attempt to decrease command and test that motor limit is not reached
	mixer_input_command(&mixer, MIXER_INPUT_G0_PITCH, 0);
	mixer_update(&mixer);

	EXPECT_EQ(false, mixer_motor_limit_reached(&mixer));

	// expect all motors to be at throttle value
	EXPECT_EQ(1500, mixer_get_motor_value(&mixer, 0));
	EXPECT_EQ(1500, mixer_get_motor_value(&mixer, 1));
	EXPECT_EQ(1500, mixer_get_motor_value(&mixer, 2));
	EXPECT_EQ(1500, mixer_get_motor_value(&mixer, 3));

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
	EXPECT_EQ(1400, mixer_get_motor_value(&mixer, 0));
	EXPECT_EQ(1500, mixer_get_servo_value(&mixer, 0));
	EXPECT_EQ(1500, mixer_get_servo_value(&mixer, 1));
	EXPECT_EQ(1500, mixer_get_servo_value(&mixer, 2));
	EXPECT_EQ(1500, mixer_get_servo_value(&mixer, 3));
	// make sure the throttle servo works too
	EXPECT_EQ(1400, mixer_get_servo_value(&mixer, 4));

	// try pitching forward 100
	mixer_input_command(&mixer, MIXER_INPUT_G0_PITCH, 100);
	mixer_input_command(&mixer, MIXER_INPUT_G0_THROTTLE, 100);
	mixer_update(&mixer);

	// expect elevator to have moved
	EXPECT_EQ(1500, mixer_get_servo_value(&mixer, 0));
	EXPECT_EQ(1500, mixer_get_servo_value(&mixer, 1));
	EXPECT_EQ(1500, mixer_get_servo_value(&mixer, 2));
	EXPECT_EQ(1600, mixer_get_servo_value(&mixer, 3));
	EXPECT_EQ(1600, mixer_get_servo_value(&mixer, 4));

	// attempt to provide full pitch and check that we get motor limit reached flag set
	mixer_input_command(&mixer, MIXER_INPUT_G0_ROLL, 500);
	mixer_update(&mixer);

	// doesn't really matter but we just test that we did not reach motor limit.
	EXPECT_EQ(false, mixer_motor_limit_reached(&mixer));

	// expect aelerons to have moved
	EXPECT_EQ(2000, mixer_get_servo_value(&mixer, 0));
	EXPECT_EQ(2000, mixer_get_servo_value(&mixer, 1));
	EXPECT_EQ(1500, mixer_get_servo_value(&mixer, 2));
	EXPECT_EQ(1600, mixer_get_servo_value(&mixer, 3));
	EXPECT_EQ(1600, mixer_get_servo_value(&mixer, 4));

	testedModes++;
}

TEST_F(MixerBasicTest, TestMixerLoadSave){
	// set up some config defaults
	rxConfig()->mincheck = 1010;
	rxConfig()->midrc = 1500;
	motorAndServoConfig()->mincommand = 1000;
	motorAndServoConfig()->minthrottle = 1050;
	motorAndServoConfig()->maxthrottle = 1850;

	_init_mixer_defaults(MIXER_AIRPLANE);

	EXPECT_EQ(1, mixer_get_motor_count(&mixer));
	EXPECT_EQ(5, mixer_get_servo_count(&mixer));

	struct motor_mixer motors[8], motors2[8];
	struct servo_mixer servos[8], servos2[8];

	memset(motors, 0, sizeof(motors));
	memset(motors2, 0, sizeof(motors2));
	memset(servos, 0, sizeof(servos));
	memset(servos2, 0, sizeof(servos2));

	struct mixer_rule_def rules[sizeof(mixer.active_rules) / sizeof(struct mixer_rule_def)];
	EXPECT_EQ(sizeof(rules), sizeof(mixer.active_rules));
	EXPECT_EQ(6, mixer.ruleCount);

	memcpy(rules, mixer.active_rules, sizeof(mixer.active_rules));

	EXPECT_EQ(mixer_save_motor_mixer(&mixer, motors), 1);
	EXPECT_EQ(mixer_save_servo_mixer(&mixer, servos), 5);
	
	mixer_clear_rules(&mixer);
	
	mixer_load_motor_mixer(&mixer, motors);
	mixer_load_servo_mixer(&mixer, servos);
	
	mixer_load_motor_mixer(&mixer, motors);
	mixer_load_servo_mixer(&mixer, servos);

	EXPECT_EQ(6, mixer.ruleCount);
	EXPECT_EQ(memcmp(rules, mixer.active_rules, sizeof(rules)), 0);

	// try saving again and compare
	mixer_save_motor_mixer(&mixer, motors2);
	mixer_save_servo_mixer(&mixer, servos2);
	
	EXPECT_EQ(memcmp(motors, motors2, sizeof(motors)), 0);
	EXPECT_EQ(memcmp(servos, servos2, sizeof(servos)), 0);
}


TEST_F(MixerBasicTest, TestAllModesTested){
	//EXPECT_EQ(MIXER_MODE_COUNT, testedModes);
}

/*
TEST_F(ChannelForwardingTest, TestForwardAuxChannelsToServosWithMaxServos)
{
    // given
    servoCount = MAX_SUPPORTED_SERVOS;

    rcData[AUX1] = 1000;
    rcData[AUX2] = 1250;
    rcData[AUX3] = 1750;
    rcData[AUX4] = 2000;

    // when
    forwardAuxChannelsToServos(MAX_SUPPORTED_SERVOS);

    // then
    uint8_t i;
    for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        EXPECT_EQ(0, servos[i].value);
    }
}

TEST_F(ChannelForwardingTest, TestForwardAuxChannelsToServosWithLessRemainingServosThanAuxChannelsToForward)
{
    // given
    servoCount = MAX_SUPPORTED_SERVOS - 2;

    rcData[AUX1] = 1000;
    rcData[AUX2] = 1250;
    rcData[AUX3] = 1750;
    rcData[AUX4] = 2000;

    // when
    forwardAuxChannelsToServos(MAX_SUPPORTED_SERVOS - 2);

    // then
    uint8_t i;
    for (i = 0; i < MAX_SUPPORTED_SERVOS - 2; i++) {
        EXPECT_EQ(0, servos[i].value);
    }

    // -1 for zero based offset
    EXPECT_EQ(1000, servos[MAX_SUPPORTED_SERVOS - 1 - 1].value);
    EXPECT_EQ(1250, servos[MAX_SUPPORTED_SERVOS - 0 - 1].value);
}
*/
class BasicMixerIntegrationTest : public ::testing::Test {
protected:
    struct servo_config servoConf[MAX_SUPPORTED_SERVOS];
    gimbalConfig_t gimbalConfig = {
        .mode = GIMBAL_MODE_NORMAL
    };

    virtual void SetUp() {
        updatedServoCount = 0;
        updatedMotorCount = 0;

        memset(mixerConfig(), 0, sizeof(*mixerConfig()));
        memset(rxConfig(), 0, sizeof(*rxConfig()));
        memset(motorAndServoConfig(), 0, sizeof(*motorAndServoConfig()));
        memset(servoProfile(), 0, sizeof(*servoProfile()));

        memset(rcData, 0, sizeof(rcData));
        memset(rcCommand, 0, sizeof(rcCommand));
        memset(&pid_output, 0, sizeof(pid_output));
        memset(customMotorMixer_arr(), 0, sizeof(*customMotorMixer_arr()));
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

/*
TEST_F(BasicMixerIntegrationTest, TestTricopterServo)
{
    // given
    rxConfig()->midrc = 1500;

    mixerConfig()->tri_unarmed_servo = 1;

    withDefaultmotorAndServoConfiguration();
    withDefaultRxConfig();

    servoConf[5].min = DEFAULT_SERVO_MIN;
    servoConf[5].max = DEFAULT_SERVO_MAX;
    servoConf[5].middle = DEFAULT_SERVO_MIDDLE;
    servoConf[5].rate = 100;
    servoConf[5].forwardFromChannel = CHANNEL_FORWARDING_DISABLED;

    configureMixer(MIXER_TRI);

    mixer_init(&default_mixer, 
		mixerConfig(),
		motor3DConfig(),
		motorAndServoConfig(),
		rxConfig(),
		rcControlsConfig(),
		servoProfile()->servoConf,
		customMotorMixer(0), MAX_SUPPORTED_MOTORS);

    // and
    pwmIOConfiguration_t pwmIOConfiguration = {
            .servoCount = 1,
            .motorCount = 3,
            .ioCount = 4,
            .pwmInputCount = 0,
            .ppmInputCount = 0,
            .ioConfigurations = {}
    };

    mixer_set_pwmio_config(&default_mixer, &pwmIOConfiguration);
    // and
	pid_output.axis[FD_YAW] = 0;

    // when
    mixer_update(&default_mixer, &pid_output);

	mixer_write_pwm(&default_mixer);

    // then
    EXPECT_EQ(1, updatedServoCount);
    EXPECT_EQ(TEST_SERVO_MID, servos[0].value);
}
*/
TEST_F(BasicMixerIntegrationTest, TestQuadMotors)
{
    // given
    withDefaultmotorAndServoConfiguration();

    configureMixer(MIXER_QUADX);

	mixer_init(&default_mixer, 
		mixerConfig(),
		motor3DConfig(),
		motorAndServoConfig(),
		rxConfig(),
		rcControlsConfig(),
		servoProfile()->servoConf,
		customMotorMixer(0), MAX_SUPPORTED_MOTORS);

    // and
	/*
    pwmIOConfiguration_t pwmIOConfiguration = {
            .servoCount = 0,
            .motorCount = 4,
            .ioCount = 4,
            .pwmInputCount = 0,
            .ppmInputCount = 0,
            .ioConfigurations = {}
    };

    mixerUsePWMIOConfiguration(&default_mixer, &pwmIOConfiguration);
*/
    // and
    memset(rcCommand, 0, sizeof(rcCommand));

    // and
    memset(&pid_output, 0, sizeof(pid_output));
	pid_output.axis[FD_YAW] = 0; 

    // when
    mixer_update(&default_mixer);

    // then
    EXPECT_EQ(4, mixer_get_motor_count(&default_mixer));

    EXPECT_EQ(TEST_MIN_COMMAND, mixer_get_motor_value(&default_mixer, 0));
    EXPECT_EQ(TEST_MIN_COMMAND, mixer_get_motor_value(&default_mixer, 1));
    EXPECT_EQ(TEST_MIN_COMMAND, mixer_get_motor_value(&default_mixer, 2));
    EXPECT_EQ(TEST_MIN_COMMAND, mixer_get_motor_value(&default_mixer, 3));
}


class CustomMixerIntegrationTest : public BasicMixerIntegrationTest {
protected:

    virtual void SetUp() {

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

/*
TEST_F(CustomMixerIntegrationTest, TestCustomMixer)
{
    // given
    enum {
        EXPECTED_SERVOS_TO_MIX_COUNT = 6,
        EXPECTED_MOTORS_TO_MIX_COUNT = 2
    };

    struct servo_mixer testServoMixer[EXPECTED_SERVOS_TO_MIX_COUNT] = {
        { SERVO_ELEVATOR, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
        { SERVO_FLAPPERON_1, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
        { SERVO_FLAPPERON_2, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
        { SERVO_RUDDER, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
        { SERVO_THROTTLE, INPUT_STABILIZED_THROTTLE, 100, 0, 0, 100, 0 },
        { SERVO_FLAPS, INPUT_RC_AUX1,  100, 0, 0, 100, 0 },
    };
    memcpy(customServoMixer_arr(), testServoMixer, sizeof(testServoMixer));

    static const struct motor_mixer testMotorMixer[EXPECTED_MOTORS_TO_MIX_COUNT] = {
        { 1.0f,  0.0f,  0.0f, -1.0f },          // LEFT
        { 1.0f,  0.0f,  0.0f,  1.0f },          // RIGHT
    };
    memcpy(customMotorMixer_arr(), testMotorMixer, sizeof(testMotorMixer));

    configureMixer(MIXER_CUSTOM_AIRPLANE);

    mixer_init(&default_mixer, 
		mixerConfig(),
		motor3DConfig(),
		motorAndServoConfig(),
		rxConfig(),
		rcControlsConfig(),
		servoProfile()->servoConf,
		customMotorMixer(0), MAX_SUPPORTED_MOTORS);


    pwmIOConfiguration_t pwmIOConfiguration = {
            .servoCount = 6,
            .motorCount = 2,
            .ioCount = 8,
            .pwmInputCount = 0,
            .ppmInputCount = 0,
            .ioConfigurations = {}
    };

    mixer_set_pwmio_config(&default_mixer, &pwmIOConfiguration);

    // and
    rcCommand[THROTTLE] = 1000;

    // and
    rcData[AUX1] = 2000;

    // and
    memset(&pid_output, 0, sizeof(pid_output));

    // when
    mixer_update(&default_mixer, &pid_output);
    mixer_write_pwm(&default_mixer);

    // then
    EXPECT_EQ(EXPECTED_MOTORS_TO_MIX_COUNT, mixer_get_motor_count(&default_mixer));

    EXPECT_EQ(TEST_MIN_COMMAND, mixer_get_motor_value(&default_mixer, 0));
    EXPECT_EQ(TEST_MIN_COMMAND, mixer_get_motor_value(&default_mixer, 1));

    EXPECT_EQ(EXPECTED_SERVOS_TO_MIX_COUNT, updatedServoCount);

    EXPECT_EQ(TEST_SERVO_MID, servos[0].value);
    EXPECT_EQ(TEST_SERVO_MID, servos[1].value);
    EXPECT_EQ(TEST_SERVO_MID, servos[2].value);
    EXPECT_EQ(TEST_SERVO_MID, servos[3].value);
    EXPECT_EQ(1000, servos[4].value); // Throttle
    EXPECT_EQ(2000, servos[5].value); // Flaps
}

*/

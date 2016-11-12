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
#include <math.h>

#include <limits.h>

extern "C" {
    #include "debug.h"

    #include <platform.h>
    #include "build_config.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/filter.h"

	#include "config/config.h"
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
    #include "flight/tilt.h"

    #include "io/rc_controls.h"

    #include "config/config.h"

    //void mixerUsePWMIOConfiguration(struct mixer *self, pwmIOConfiguration_t *pwmIOConfiguration);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

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
	uint8_t testFeatureMask;

	float applyBiQuadFilter(float sample, biquad_t *state) {UNUSED(state);return sample;}
	void BiQuadNewLpf(float filterCutFreq, biquad_t *newState, uint32_t refreshRate) {UNUSED(filterCutFreq);UNUSED(newState);UNUSED(refreshRate);}


	bool feature(uint32_t mask) {
		return (mask & testFeatureMask);
	}
	bool rcModeIsActive(boxId_e modeId) { return rcModeActivationMask & (1 << modeId); }
}

TEST(TiltComensator, CompensationTest){
	struct tilt_config conf;
	struct tilt_input_params input;
	struct tilt_output_params output;
	conf.mode = MIXER_TILT_MODE_DYNAMIC,
	conf.control_channel = AUX1,
	conf.compensation_flags = MIXER_TILT_COMPENSATE_THRUST | MIXER_TILT_COMPENSATE_TILT | MIXER_TILT_COMPENSATE_BODY,
	input.motor_pitch_dd = 0;
	input.body_pitch_dd = 0;
	input.roll = 0;
	input.pitch = 0;
	input.yaw = 0;
	input.throttle = 500;

	tilt_calculate_compensation(&conf, &input, &output);

	EXPECT_EQ(input.throttle, output.throttle);
	EXPECT_EQ(0, output.roll);
	EXPECT_EQ(0, output.pitch);
	EXPECT_EQ(0, output.yaw);

	// this should cause the tilt code to generate no compensation since props are pointing up
	input.body_pitch_dd = -100;
	input.motor_pitch_dd = -100;
	tilt_calculate_compensation(&conf, &input, &output);

	EXPECT_EQ(input.throttle, output.throttle);
	EXPECT_EQ(0, output.roll);
	EXPECT_EQ(0, output.pitch);
	EXPECT_EQ(0, output.yaw);

	// now tilt only motors forward
	input.body_pitch_dd = 0;
	input.motor_pitch_dd = -350;
	tilt_calculate_compensation(&conf, &input, &output);

	// test throttle compensation so that throttle is scaled accordingly
	EXPECT_EQ((int16_t)(input.throttle / cosf(35.0f * M_PIf / 180.0f)), output.throttle);
	EXPECT_EQ(0, output.roll);
	EXPECT_EQ(0, output.pitch);
	EXPECT_EQ(0, output.yaw);

	// test yaw/roll compensation
	// if props are 90 deg then roll command becomes yaw
	// if props are -90 then yaw is opposite to roll command
	// this needs to work regardless of body pitch
	input.motor_pitch_dd = 900;
	input.roll = 450;
	tilt_calculate_compensation(&conf, &input, &output);

	EXPECT_EQ(0, output.roll);
	EXPECT_EQ(0, output.pitch);
	EXPECT_EQ(450, output.yaw);

	input.motor_pitch_dd = -900;
	input.roll = 450;
	tilt_calculate_compensation(&conf, &input, &output);

	EXPECT_EQ(0, output.roll);
	EXPECT_EQ(0, output.pitch);
	EXPECT_EQ(-450, output.yaw);

	// if we instead yaw 45 degrees then we should see negative roll when props are facing backwards
	input.roll = 0;
	input.yaw = 450;
	tilt_calculate_compensation(&conf, &input, &output);

	EXPECT_EQ(-450, output.roll);
	EXPECT_EQ(0, output.pitch);
	EXPECT_EQ(0, output.yaw);

	// test forward facing props
	input.motor_pitch_dd = 900;
	input.roll = 0;
	input.yaw = 450;
	tilt_calculate_compensation(&conf, &input, &output);

	EXPECT_EQ(450, output.roll);
	EXPECT_EQ(0, output.pitch);
	EXPECT_EQ(0, output.yaw);

	//EXPECT_EQ(1, 0);
}


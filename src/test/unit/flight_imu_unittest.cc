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

#define BARO

extern "C" {
    #include <platform.h>
    #include "build_config.h"
    #include "debug.h"

    #include "common/axis.h"
    #include "common/maths.h"

	#include "config/config.h"

    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "drivers/compass.h"

    #include "sensors/gyro.h"
    #include "sensors/compass.h"
    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"

    #include "config/config.h"

    #include "rx/rx.h"

    #include "flight/mixer.h"
    #include "flight/anglerate.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

void input_imu_accel(struct imu *self, int16_t x, int16_t y, int16_t z){
	imu_reset(self);
	imu_input_accelerometer(self, x, y, z);
	imu_input_gyro(self, 0, 0, 0);
	for(unsigned int c = 0; c < 20000UL; c++){
		imu_update(self, 0.02);
	}
}

class FlightImuTest : public ::testing::Test {
protected:
    virtual void SetUp() {
		mock_system_reset();

		config_reset(&config);

		config.imu.dcm_kp = 500;
		config.imu.looptime = 2000;
		config.imu.gyroSync = 1;
		config.imu.gyroSyncDenominator = 1;
		config.imu.small_angle = 25;
		config.imu.max_angle_inclination = 500;

		imu_init(&imu, &config);
    }
	struct config config;
	struct imu imu;
};

/**
 * @ingroup IMU
 * @page IMU
 *
 * - Test that imu leveling is detected correctly.
 */

TEST_F(FlightImuTest, TestImuLevel){
	input_imu_accel(&imu, 0, 0, 1024);
	EXPECT_TRUE(imu_is_leveled(&imu, 10));
	input_imu_accel(&imu, 724, 0, 724);
	EXPECT_FALSE(imu_is_leveled(&imu, 10));
	input_imu_accel(&imu, 0, 0, 1024);
	EXPECT_TRUE(imu_is_leveled(&imu, 10));
}

/**
 * @ingroup IMU
 * @page IMU
 *
 * - Make sure that different acc readings properly align the imu.
 */

TEST_F(FlightImuTest, TestEulerAngleCalculation){
	imu_set_gyro_scale(&imu, 1.0f/16.4f);
	imu_set_acc_scale(&imu, 1024);
	// accelerometer reads linear acceleration - g so and also it is upside down.
	// rotations are positive clockwise and negative ccw when looking down an axis

	// so when it is leveled it gives reading perpendecular to g which is positive z
    input_imu_accel(&imu, 0, 0, 1024);
    EXPECT_FLOAT_EQ(imu_get_pitch_dd(&imu), 0);
    EXPECT_FLOAT_EQ(imu_get_yaw_dd(&imu), 0);

	// when it is tilted nose up it will give reading along positive x since gravity is along negative x
    input_imu_accel(&imu, 1024, 0, 0);
    EXPECT_FLOAT_EQ(imu_get_roll_dd(&imu), 0);
    EXPECT_FLOAT_EQ(imu_get_pitch_dd(&imu), -900);
    EXPECT_FLOAT_EQ(imu_get_yaw_dd(&imu), 0);

	// since y is to the left, rotating cw 90 deg around x will give acceleration pointing upwards and gravity in -y direction
    input_imu_accel(&imu, 0, 1024, 0);
    EXPECT_FLOAT_EQ(imu_get_roll_dd(&imu), 900);
    EXPECT_FLOAT_EQ(imu_get_pitch_dd(&imu), 0);
    EXPECT_FLOAT_EQ(imu_get_yaw_dd(&imu), 0);

	// pitching up -45 degrees around y will give positive x and z reading
    input_imu_accel(&imu, 724, 0, 724);
    EXPECT_FLOAT_EQ(imu_get_roll_dd(&imu), 0);
    EXPECT_FLOAT_EQ(imu_get_pitch_dd(&imu), -450);
    EXPECT_FLOAT_EQ(imu_get_yaw_dd(&imu), 0);

	// rolling cw around x will give positive y and z reading
    input_imu_accel(&imu, 0, 724, 724);
    EXPECT_FLOAT_EQ(imu_get_roll_dd(&imu), 450);
    EXPECT_FLOAT_EQ(imu_get_pitch_dd(&imu), 0);
    EXPECT_FLOAT_EQ(imu_get_yaw_dd(&imu), 0);

	// this one is a pretty weird test. It does not really work because it involves yaw which we can not estimate using only the accelerometer
	// TODO: this needs more investigation (for now things seem to work though)
    input_imu_accel(&imu, 724, 724, 0);
    EXPECT_FLOAT_EQ(imu_get_roll_dd(&imu), 843);
    EXPECT_FLOAT_EQ(imu_get_pitch_dd(&imu), -450);
    EXPECT_FLOAT_EQ(imu_get_yaw_dd(&imu), 450);
}


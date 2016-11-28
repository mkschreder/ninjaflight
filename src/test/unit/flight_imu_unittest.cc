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
    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"

    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "drivers/compass.h"

    #include "sensors/gyro.h"
    #include "sensors/compass.h"
    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"

    #include "config/runtime_config.h"
    #include "config/config.h"

    #include "io/rc_controls.h"

    #include "rx/rx.h"

    #include "io/rc_controls.h"

    #include "flight/mixer.h"
    #include "flight/anglerate.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
void imuComputeRotationMatrix(void);
void imuUpdateEulerAngles(void);

struct imu default_imu; 
int16_t cycleTime = 2000;

}

void input_imu_accel(int16_t x, int16_t y, int16_t z){
	imu_reset(&default_imu);
	imu_input_accelerometer(&default_imu, x, y, z);
	imu_input_gyro(&default_imu, 0, 0, 0);
	for(unsigned int c = 0; c < 10000UL; c++){
		imu_update(&default_imu, 0.01);
	}
/*
    if (initialRoll > 1800) initialRoll -= 3600;
    if (initialPitch > 1800) initialPitch -= 3600;
    if (initialYaw > 1800) initialYaw -= 3600;

    float cosRoll = cosf(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);
    float sinRoll = sinf(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);

    float cosPitch = cosf(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);
    float sinPitch = sinf(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);

    float cosYaw = cosf(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);
    float sinYaw = sinf(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);

    q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

    imuComputeRotationMatrix();
*/
}

TEST(FlightImuTest, TestEulerAngleCalculation){
	imuConfig()->dcm_kp = 2500;
    imuConfig()->looptime = 2000;
    imuConfig()->gyroSync = 1;
    imuConfig()->gyroSyncDenominator = 1;
    imuConfig()->small_angle = 25;
    imuConfig()->max_angle_inclination = 500;

	imu_init(&default_imu,
		imuConfig(),
		accelerometerConfig(),
		throttleCorrectionConfig()
	);

	imu_set_gyro_scale(&default_imu, 1.0f/16.4f);
	imu_set_acc_scale(&default_imu, 1024);
	// accelerometer reads linear acceleration - g so and also it is upside down.
	// rotations are positive clockwise and negative ccw when looking down an axis

	// so when it is leveled it gives reading perpendecular to g which is positive z
    input_imu_accel(0, 0, 1024);
    EXPECT_FLOAT_EQ(imu_get_pitch_dd(&default_imu), 0);
    EXPECT_FLOAT_EQ(imu_get_yaw_dd(&default_imu), 0);

	// when it is tilted nose up it will give reading along positive x since gravity is along negative x
    input_imu_accel(1024, 0, 0);
    EXPECT_FLOAT_EQ(imu_get_roll_dd(&default_imu), 0);
    EXPECT_FLOAT_EQ(imu_get_pitch_dd(&default_imu), -900);
    EXPECT_FLOAT_EQ(imu_get_yaw_dd(&default_imu), 0);

	// since y is to the left, rotating cw 90 deg around x will give acceleration pointing upwards and gravity in -y direction
    input_imu_accel(0, 1024, 0);
    EXPECT_FLOAT_EQ(imu_get_roll_dd(&default_imu), 900);
    EXPECT_FLOAT_EQ(imu_get_pitch_dd(&default_imu), 0);
    EXPECT_FLOAT_EQ(imu_get_yaw_dd(&default_imu), 0);

	// pitching up -45 degrees around y will give positive x and z reading
    input_imu_accel(724, 0, 724);
    EXPECT_FLOAT_EQ(imu_get_roll_dd(&default_imu), 0);
    EXPECT_FLOAT_EQ(imu_get_pitch_dd(&default_imu), -450);
    EXPECT_FLOAT_EQ(imu_get_yaw_dd(&default_imu), 0);

	// rolling cw around x will give positive y and z reading
    input_imu_accel(0, 724, 724);
    EXPECT_FLOAT_EQ(imu_get_roll_dd(&default_imu), 450);
    EXPECT_FLOAT_EQ(imu_get_pitch_dd(&default_imu), 0);
    EXPECT_FLOAT_EQ(imu_get_yaw_dd(&default_imu), 0);

	// this one is a pretty weird test. It does not really work because it involves yaw which we can not estimate using only the accelerometer
	// TODO: this needs more investigation (for now things seem to work though)
    input_imu_accel(724, 724, 0);
    EXPECT_FLOAT_EQ(imu_get_roll_dd(&default_imu), 843);
    EXPECT_FLOAT_EQ(imu_get_pitch_dd(&default_imu), -450);
    EXPECT_FLOAT_EQ(imu_get_yaw_dd(&default_imu), 450);
}

// STUBS

extern "C" {
uint32_t rcModeActivationMask;
int16_t rcCommand[4];
int16_t rcData[RX_MAX_SUPPORTED_RC_CHANNELS];
// TODO: proper way to do this is to write a mock receiver
int16_t rc_get_channel_value(uint8_t id){ return rcData[id]; }
void rc_set_channel_value(uint8_t id, int16_t value){ rcData[id] = value; }

acc_t acc;
int16_t heading;
gyro_t gyro;
int32_t magADC[XYZ_AXIS_COUNT];
int32_t BaroAlt;
int16_t debug[DEBUG16_VALUE_COUNT];

uint8_t stateFlags;
uint16_t flightModeFlags;
uint8_t armingFlags;

int32_t sonarAlt;
int16_t sonarCfAltCm;
int16_t sonarMaxAltWithTiltCm;
int32_t accADC[XYZ_AXIS_COUNT];
int32_t gyroADC[XYZ_AXIS_COUNT];

uint16_t GPS_speed;
uint16_t GPS_ground_course;
uint8_t GPS_numSat;

float magneticDeclination = 0.0f;

bool rcModeIsActive(boxId_e modeId) { return rcModeActivationMask & (1 << modeId); }

void gyroUpdate(void) {};
bool sensors(uint32_t mask)
{
    UNUSED(mask);
    return false;
};
void updateAccelerationReadings(rollAndPitchTrims_t *rollAndPitchTrims)
{
    UNUSED(rollAndPitchTrims);
}

uint32_t micros(void) { return 0; }
uint32_t millis(void) { return 0; }
bool isBaroCalibrationComplete(void) { return true; }
void performBaroCalibrationCycle(void) {}
int32_t baroCalculateAltitude(void) { return 0; }
}

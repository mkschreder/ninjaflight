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

#include <platform.h>

#include "build_config.h"

//#define DEBUG_ALTITUDE_HOLD

#define BARO

extern "C" {
    #include "debug.h"

    #include "common/axis.h"
    #include "common/maths.h"

    #include "config/parameter_group_ids.h"
    #include "config/parameter_group.h"

    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"

    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"

    #include "io/rc_controls.h"

    #include "rx/rx.h"

    #include "flight/mixer.h"
    #include "flight/anglerate.h"
    #include "flight/altitudehold.h"

    #include "config/runtime_config.h"
    #include "config/config.h"
	#include "config/rc_controls.h"
	#include "config/altitudehold.h"
	#include "config/mixer.h"

/*
    PG_REGISTER_PROFILE(struct pid_config, pidProfile, PG_PID_PROFILE, 0);
    PG_REGISTER_PROFILE(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);
    PG_REGISTER_PROFILE(barometerConfig_t, barometerConfig, PG_BAROMETER_CONFIG, 0);

    PG_REGISTER(motorAndServoConfig_t, motorAndServoConfig, PG_MOTOR_AND_SERVO_CONFIG, 0);

	PG_REGISTER_WITH_RESET_TEMPLATE(airplaneConfig_t, airplaneConfig, PG_AIRPLANE_ALT_HOLD_CONFIG, 0);

	PG_RESET_TEMPLATE(airplaneConfig_t, airplaneConfig,
		.fixedwing_althold_dir = 1,
	);
*/

	struct imu default_imu; 
    extern uint32_t rcModeActivationMask;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

#define DOWNWARDS_THRUST true
#define UPWARDS_THRUST false


extern "C" {
    bool isThrustFacingDownwards(union attitude_euler_angles * attitude);
    uint16_t calculateTiltAngle(union attitude_euler_angles * attitude);
}

typedef struct inclinationExpectation_s {
    union attitude_euler_angles attitude;
    bool expectDownwardsThrust;
} inclinationExpectation_t;

/*
// TODO: this test needs to use imu
TEST(AltitudeHoldTest, IsThrustFacingDownwards)
{
    // given

    inclinationExpectation_t inclinationExpectations[] = {
            { {{    0,    0,    0 }}, DOWNWARDS_THRUST },
            { {{  799,  799,    0 }}, DOWNWARDS_THRUST },
            { {{  800,  799,    0 }}, UPWARDS_THRUST },
            { {{  799,  800,    0 }}, UPWARDS_THRUST },
            { {{  800,  800,    0 }}, UPWARDS_THRUST },
            { {{  801,  801,    0 }}, UPWARDS_THRUST },
            { {{ -799, -799,    0 }}, DOWNWARDS_THRUST },
            { {{ -800, -799,    0 }}, UPWARDS_THRUST },
            { {{ -799, -800,    0 }}, UPWARDS_THRUST },
            { {{ -800, -800,    0 }}, UPWARDS_THRUST },
            { {{ -801, -801,    0 }}, UPWARDS_THRUST }
    };
    uint8_t testIterationCount = sizeof(inclinationExpectations) / sizeof(inclinationExpectation_t);

    // expect

    for (uint8_t index = 0; index < testIterationCount; index ++) {
        inclinationExpectation_t *angleInclinationExpectation = &inclinationExpectations[index];
#ifdef DEBUG_ALTITUDE_HOLD
        printf("iteration: %d\n", index);
#endif
        bool result = isThrustFacingDownwards(&angleInclinationExpectation->attitude);
        EXPECT_EQ(angleInclinationExpectation->expectDownwardsThrust, result);
    }
}
*/
#if 0
// TODO: reenable althold unit test and rewrite it properly
TEST(AltitudeHoldTest, applyMultirotorAltHold)
{
    // given

    memset(motorAndServoConfig(), 0, sizeof(motorAndServoConfig_t));
    motorAndServoConfig()->minthrottle = 1150;
    motorAndServoConfig()->maxthrottle = 1850;
    memset(rcControlsConfig(), 0, sizeof(rcControlsConfig_t));
    rcControlsConfig()->alt_hold_deadband = 40;
    
    rc_set_channel_value(THROTTLE, 1400);
    rcCommand[THROTTLE] = 1500;
    rcModeActivationMask |= (1 << BOXBARO);
    althold_update(&althold);
    
    // when
    apply_althold(&althold);
    
    // expect
    EXPECT_EQ(1500, rcCommand[THROTTLE]);
    
    // and given
    rcControlsConfig()->alt_hold_fast_change = 1;
    
    // when
    applyAltHold();
    
    // expect
    EXPECT_EQ(1500, rcCommand[THROTTLE]);
}
#endif
// STUBS

extern "C" {
uint32_t rcModeActivationMask;
bool rcModeIsActive(boxId_e modeId) { return rcModeActivationMask & (1 << modeId); }
int16_t rcCommand[4];
int16_t rcData[RX_MAX_SUPPORTED_RC_CHANNELS];
// TODO: proper way to do this is to write a mock receiver
int16_t rc_get_channel_value(uint8_t id){ return rcData[id]; }
void rc_set_channel_value(uint8_t id, int16_t value){ rcData[id] = value; }
uint32_t accTimeSum ;        // keep track for integration of acc
int accSumCount;
float accVelScale;

float imu_get_velocity_integration_time(struct imu *self){ UNUSED(self); return 0.001;}
float imu_get_est_vertical_vel_cms(struct imu *self){ UNUSED(self); return 0; }
void imu_reset_velocity_estimate(struct imu *self){ UNUSED(self); }
float imu_get_avg_vertical_accel_cmss(struct imu *self){ UNUSED(self); return 0; }

int16_t imu_get_roll_dd(struct imu *self){ return self->attitude.values.roll; }
int16_t imu_get_pitch_dd(struct imu *self){ return self->attitude.values.pitch; }
//uint16_t acc_1G;
//int16_t heading;
//gyro_t gyro;
//int32_t accSum[XYZ_AXIS_COUNT];
//int16_t magADC[XYZ_AXIS_COUNT];
int32_t BaroAlt;
int16_t debug[DEBUG16_VALUE_COUNT];

uint8_t stateFlags;
uint16_t flightModeFlags;
uint8_t armingFlags;

int32_t sonarAlt;
int16_t sonarCfAltCm;
int16_t sonarMaxAltWithTiltCm;

uint16_t enableFlightMode(flightModeFlags_e mask)
{
    return flightModeFlags |= (mask);
}

uint16_t disableFlightMode(flightModeFlags_e mask)
{
    return flightModeFlags &= ~(mask);
}

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

void imuResetAccelerationSum(void) {};

uint32_t micros(void) { return 0; }
bool isBaroCalibrationComplete(void) { return true; }
void performBaroCalibrationCycle(void) {}
int32_t baroCalculateAltitude(void) { return 0; }
}

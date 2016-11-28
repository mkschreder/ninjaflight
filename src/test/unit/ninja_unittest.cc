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

	#include "ninja.h"
	#include "ninja_input.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

/**
 * @defgroup NINJASPEC Guarantees
 * @ingroup NINJA
 *
 * @page NINJASPEC
 * @ingroup NINJASPEC
 *
 * This is a summary of automatic tests that are done against the flight
 * controller in full. Each requirement is always backed by a unit test.
 */

void reset_env(void){
	imuConfig()->dcm_kp = 2500;
    imuConfig()->looptime = 2000;
    imuConfig()->gyroSync = 1;
    imuConfig()->gyroSyncDenominator = 1;
    imuConfig()->small_angle = 25;
    imuConfig()->max_angle_inclination = 500;

	rxConfig()->mincheck = 1010;

	// default frame config
	mixerConfig()->mixerMode = MIXER_QUADX;

	// mixer and motors output
	motorAndServoConfig()->mincommand = 1010;
	motorAndServoConfig()->minthrottle = 1020;
	motorAndServoConfig()->maxthrottle = 2000;
}

void run_loop(struct ninja *self, uint32_t ms){
	for(uint32_t c = 0; c < ms; c++){
		ninja_heartbeat(self);
		usleep(1000);
	}
}

/**
 * @page NINJASPEC
 * @ingroup NINJASPEC
 *
 * - TODO: Flight controller automatically detects if sensors are connected by
 * inspecting return values from system calls that read the sensors. If sensors
 * are disconnected or otherwise not available then system calls should return
 * -ENOENT (or other negative value). On success system calls will return zero.
 */
TEST(InsUnitTest, ArmingTest){
	struct ninja ninja;

	reset_env();

	ninja_init(&ninja, mock_syscalls());

	run_loop(&ninja, 5000);

	// double check that all motors are at mincommand (we are disarmed)
	/*
	EXPECT_EQ(motorAndServoConfig()->mincommand, mock_motor_pwm[0]);
	EXPECT_EQ(motorAndServoConfig()->mincommand, mock_motor_pwm[1]);
	EXPECT_EQ(motorAndServoConfig()->mincommand, mock_motor_pwm[2]);
	EXPECT_EQ(motorAndServoConfig()->mincommand, mock_motor_pwm[3]);
	*/
}

extern "C" {
/*
	uint32_t gyro_sync_get_looptime(void){ return 1000; }
	int16_t rc_get_channel_value(uint8_t){ return 1500; }
	void updateTransponder(void){}
	void handleSerial(void){}
	void updateRx(uint32_t){}
	bool shouldProcessRx(uint32_t){ return true; }
	void gpsThread(void) {}
	void updateGpsIndicator(void) { }
	void calculateEstimatedAltitude(void){}
	void updateDisplay(void){}
	void updateLedStrip(void){}
*/
}

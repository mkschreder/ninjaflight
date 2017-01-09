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
#include <stddef.h>

extern "C" {
#include "config/config.h"
#include "io/beeper.h"
/*
#include "platform.h"
#include "build_config.h"

#include "config/config.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/color.h"
#include "common/utils.h"

#include "flight/anglerate.h"

#include "drivers/sensor.h"
#include "drivers/timer.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"

#include "flight/rate_profile.h"
#include "io/rc_adjustments.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/transponder_ir.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"

#include "flight/mixer.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"
#include "flight/altitudehold.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky.h"
#include "telemetry/hott.h"

#include "config/config.h"
#include "config/feature.h"
#include "config/profile.h"
#include "config/frsky.h"

#include "platform.h"
*/
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

class BeeperTest : public ::testing::Test {
protected:
    virtual void SetUp() {
		mock_system_reset();
		// use manual clocking so we can check timing
		mock_system_clock_mode(MOCK_CLOCK_MANUAL);

		config_reset(&config);

		beeper_init(&beeper, mock_syscalls());
    }
	struct config_store config;
	struct beeper beeper;
};

TEST_F(BeeperTest, BeeperShortBeepTest){
	mock_time_micros = 0;
	EXPECT_FALSE(mock_beeper_is_on);
	// RX set is a short 100ms beep
	beeper_start(&beeper, BEEPER_RX_SET);
	beeper_update(&beeper);
	EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros = 99000;
	beeper_update(&beeper);
	EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros = 101000;
	beeper_update(&beeper);
	EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros = 201000;
	beeper_update(&beeper);
	EXPECT_FALSE(mock_beeper_is_on);
}

TEST_F(BeeperTest, BeeperPrioOverride){
	beeper_start(&beeper, BEEPER_RX_SET);
	beeper_update(&beeper);
	EXPECT_TRUE(mock_beeper_is_on);
	// try overriding with a higher priority beep
	EXPECT_TRUE(beeper_start(&beeper, BEEPER_RX_LOST));
	mock_time_micros = 200000;
	beeper_update(&beeper);
	EXPECT_TRUE(mock_beeper_is_on);
	// this beep is 500ms beep + 500ms pause
	mock_time_micros = 699000;
	beeper_update(&beeper);
	EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros = 701000;
	beeper_update(&beeper);
	EXPECT_FALSE(mock_beeper_is_on);
}

TEST_F(BeeperTest, BeeperSameNoStart){
	beeper_start(&beeper, BEEPER_GYRO_CALIBRATED);
	mock_time_micros = 0;
	beeper_update(&beeper);
	EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros = 201000;
	beeper_update(&beeper);
	EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros = 299000;
	beeper_update(&beeper);
	EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros = 302000;
	beeper_update(&beeper);
	EXPECT_TRUE(mock_beeper_is_on);
	EXPECT_FALSE(beeper_start(&beeper, BEEPER_GYRO_CALIBRATED));
	EXPECT_FALSE(beeper_start(&beeper, BEEPER_RX_SET));
	EXPECT_FALSE(beeper_start(&beeper, BEEPER_RX_LOST_LANDING));
	// all of the above should fail because we have higher priority beep
	// so we continue to check that we have the right sequence
	mock_time_micros = 503000;
	beeper_update(&beeper);
	EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros = 604000;
	beeper_update(&beeper);
	EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros = 805000;
	beeper_update(&beeper);
	EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros = 906000;
	beeper_update(&beeper);
	EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros = 1200000;
	beeper_update(&beeper);
	EXPECT_FALSE(mock_beeper_is_on);
}

TEST_F(BeeperTest, BeeperMorseTest){
	// start a noninterruptible beep
	beeper_start(&beeper, BEEPER_ACC_CALIBRATION);
	mock_time_micros = 0;
	beeper_update(&beeper);
	// writing morse now will defer the morse text till after the current beep
	beeper_write(&beeper, "AB1 +N");
	mock_time_micros += 49000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 52000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	// now the morse code should come
	// starting with a dot + dash (A)
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	// space between letters 3 units
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	// B
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	// space between letters
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	// 1
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	// pause
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	// word pause
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	// skip + and go to N
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_TRUE(mock_beeper_is_on);
	// done
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
	mock_time_micros += 51000; beeper_update(&beeper); EXPECT_FALSE(mock_beeper_is_on);
}

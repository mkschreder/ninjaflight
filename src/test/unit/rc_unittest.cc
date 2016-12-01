/*
 * This file is part of Ninjaflight.
 *
 * Unit testing and docs: Martin Schröder <mkschreder.uk@gmail.com>
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

/**
 * @page RX
 * @ingroup RX
 * This is a summary of automatic tests that are done against the receiver
 * module to guarantee that the module behaves according to the requirements
 * set forth below.
 */


#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

extern "C" {
    #include <platform.h>

    #include "config/config.h"
    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"

    #include "rx/rx.h"
	#include "flight/failsafe.h"
    #include "common/maths.h"

	#include "ninja.h"
    extern uint32_t rcModeActivationMask;

    bool isPulseValid(uint16_t pulseDuration);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

#define DE_ACTIVATE_ALL_BOXES   0

class RcTest : public ::testing::Test {
protected:
    virtual void SetUp() {
		mock_system_reset();

		rx_init(&rx, mock_syscalls());
		rx_set_type(&rx, RX_PPM);
		rx_remap_channels(&rx, "AERT1234");
		rc_init(&rc, &rx, NULL);

		// we use auto mode by default
		for(int c = 0; c < RX_MAX_SUPPORTED_RC_CHANNELS; c++){
			failsafeChannelConfigs(c)->mode = RX_FAILSAFE_MODE_AUTO;
		}
    }
	void rc_run(uint32_t ms){
		while(ms > 0){
			rx_update(&rx);
			rc_update(&rc);
			usleep(1000);
			ms--;
		}
	}
	struct rx rx;
	struct rc rc;
};

/**
 * @page RC
 * @ingroup RC
 *
 * - When RC sticks are in THR_LO + YAW_HI + PIT_CE + ROL_CE the
 * RC_KEY_STICK_ARM goes into PRESSED state and should go back to RELEASED
 * state when sticks are centered.
 */
TEST_F(RcTest, TestStartupValues){
	// setting all zeros is like saying that rx is disconnected
	memset(mock_rc_pwm, 0, sizeof(mock_rc_pwm));

	EXPECT_TRUE(rc_key_state(&rc, RC_KEY_STICK_ARM) == RC_KEY_RELEASED);

	mock_rc_pwm[ROLL] = 1500;
	mock_rc_pwm[PITCH] = 1500;
	mock_rc_pwm[YAW] = 2000;
	mock_rc_pwm[THROTTLE] = 1000;

	rc_run(5);

	EXPECT_TRUE(rc_key_state(&rc, RC_KEY_STICK_ARM) == RC_KEY_PRESSED);
}

/**
 * @page RC
 * @ingroup RC
 *
 * - When any key is triggered the event callback of the event listener should
 * be called as well.
 **/

static int _arm_count = 0;
static int _evt_count = 0;
void _arm_event(struct rc_event_listener *evt, rc_key_t key, rc_key_state_t state){
	(void)evt;
	if(key == RC_KEY_STICK_ARM && state == RC_KEY_PRESSED) _arm_count++;
	_evt_count++;
}

TEST_F(RcTest, TestKeyEvents){
	// setting all zeros is like saying that rx is disconnected
	memset(mock_rc_pwm, 0, sizeof(mock_rc_pwm));

	struct rc_event_listener evl = {
		.on_key_state = _arm_event,
		.on_key_repeat = NULL
	};
	rc.evl = &evl;

	EXPECT_TRUE(rc_key_state(&rc, RC_KEY_STICK_ARM) == RC_KEY_RELEASED);

	mock_rc_pwm[ROLL] = 1500;
	mock_rc_pwm[PITCH] = 1500;
	mock_rc_pwm[YAW] = 2000;
	mock_rc_pwm[THROTTLE] = 1000;

	rc_run(5);

	EXPECT_TRUE(rc_key_state(&rc, RC_KEY_STICK_ARM) == RC_KEY_PRESSED);
	EXPECT_EQ(1, _evt_count);
	EXPECT_EQ(1, _arm_count);
}


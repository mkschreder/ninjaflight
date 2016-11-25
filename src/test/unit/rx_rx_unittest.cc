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
    #include <platform.h>

    #include "config/config.h"
    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"

    #include "rx/rx.h"
	#include "flight/failsafe.h"
    #include "io/rc_controls.h"
    #include "common/maths.h"

	#include "ninja.h"
    extern uint32_t rcModeActivationMask;

    bool isPulseValid(uint16_t pulseDuration);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

#define DE_ACTIVATE_ALL_BOXES   0

void rx_run(struct rx *rx, uint32_t ms){
	while(ms > 0){
		rx_update(rx);
		usleep(1000);
		ms--;
	}
}

class RxTest : public ::testing::Test {
protected:
    virtual void SetUp() {
		mock_system_reset();

		rx_init(&rx, mock_syscalls());
		rx_set_type(&rx, RX_PPM);
		rx_remap_channels(&rx, "AERT1234");

		// we use auto mode by default
		for(int c = 0; c < RX_MAX_SUPPORTED_RC_CHANNELS; c++){
			failsafeChannelConfigs(c)->mode = RX_FAILSAFE_MODE_AUTO;
		}
    }
	struct rx rx;
};

/**
 * RX should output data on 16 channels. If hardware supports less then rx
 * should output failsafe values on the remaining channels. If supplied channel
 * is out of range then rx should output *midrc*.
 */
TEST_F(RxTest, TestStartupValues){
	// setting all zeros is like saying that rx is disconnected
	memset(mock_rc_pwm, 0, sizeof(mock_rc_pwm));

	rx_run(&rx, 5);

	// we do not have signal
	EXPECT_FALSE(rx_has_signal(&rx));
	// and rx is not healthy
	EXPECT_FALSE(rx_is_healthy(&rx));

	// use less channels than max just to check failsafe function
	for(int c = 0; c < RX_MAX_SUPPORTED_RC_CHANNELS; c++){
		if(c == 3) EXPECT_EQ(1000, rx_get_channel(&rx, c));
		else EXPECT_EQ(rxConfig()->midrc, rx_get_channel(&rx, c));
	}
}

/**
 * On startup all receiver channels should return failsafe values. As new
 * channels are connected rx will return the received values. However as any of
 * the channels lose signal rx should hold the value for a timeout and then
 * mark the rx as not heathy and return failsafe for that single channel. As
 * the channel is connected again, rx should become heathy. RX should report
 * that it has signal until all channels are lost.
 */
TEST_F(RxTest, TestFailsafeOnSingleChannelLoss){
	// run it for the first time
	rx_run(&rx, 5);

	// connect a 6 channel receiver
	for(int c = 0; c < 6; c++){
		mock_rc_pwm[c] = 1200;
	}

	rx_run(&rx, 1);

	// we should be healthy and have signal right away
	EXPECT_TRUE(rx_has_signal(&rx));
	EXPECT_TRUE(rx_is_healthy(&rx));

	// ensure rx is reporting correct value for the channel
	EXPECT_EQ(1200, rx_get_channel(&rx, 1));

	// disconnect one channel
	mock_rc_pwm[1] = 0;

	rx_run(&rx, 1);

	// we should still be valid since we do not give up right away
	EXPECT_TRUE(rx_has_signal(&rx));
	EXPECT_TRUE(rx_is_healthy(&rx));

	rx_run(&rx, MAX_INVALID_PULSE_TIME);

	// after this we should still have signal but rx is not healthy
	EXPECT_TRUE(rx_has_signal(&rx));
	EXPECT_FALSE(rx_is_healthy(&rx));

	// should be failsafe
	EXPECT_EQ(rxConfig()->midrc, rx_get_channel(&rx, 1));

	// now reconnect the signal
	mock_rc_pwm[1] = 1000;
	rx_run(&rx, 1);
	// should be both healthy and have signal
	EXPECT_TRUE(rx_has_signal(&rx));
	EXPECT_TRUE(rx_is_healthy(&rx));
}

/**
 * Upon signal loss (all channels first valid then invalid) all channels should
 * be outputting failsafe values.
 */
TEST_F(RxTest, TestSignalLoss){
	// use less channels than max just to check failsafe function
	for(int c = 0; c < 6; c++){
		mock_rc_pwm[c] = 1000 + c * 10;
	}

	rx_run(&rx, 1);

	for(int c = 0; c < 6; c++){
		EXPECT_EQ(1000 + c * 10, rx_get_channel(&rx, c));
	}

	// should have signal
	EXPECT_TRUE(rx_has_signal(&rx));

	for(int c = 0; c < 6; c++){
		mock_rc_pwm[c] = 0;
	}

	rx_run(&rx, 10);

	// should still have signal
	EXPECT_TRUE(rx_has_signal(&rx));

	rx_run(&rx, MAX_INVALID_PULSE_TIME);

	// now signal should be lost
	EXPECT_FALSE(rx_has_signal(&rx));

	// check that all channels are failsafe
	for(int c = 0; c < 6; c++){
		if(c == THROTTLE) EXPECT_EQ(1000, rx_get_channel(&rx, c));
		else EXPECT_EQ(rxConfig()->midrc, rx_get_channel(&rx, c));
	}

	// reconnect all channels
	for(int c = 0; c < 6; c++){
		mock_rc_pwm[c] = 1000;
	}

	rx_run(&rx, 1);

	EXPECT_TRUE(rx_has_signal(&rx));
}

/**
 * Each channel shall have configurable failsafe that can output one of three
 * alternatives: 1) auto: all channels are set to midrc except throttle. 2)
 * hold: holds old channel information. 3) set: failsafe returnes a user
 * configurable value. For safety reasons, non-aux channels should only support
 * AUTO mode.
 */
TEST_F(RxTest, TestFailsafeModes){
	// connect a receiver
	for(int c = 0; c < 6; c++){
		mock_rc_pwm[c] = 1600;
	}

	rx_run(&rx, 1);

	EXPECT_EQ(1600, rx_get_channel(&rx, AUX1));
	EXPECT_EQ(1600, rx_get_channel(&rx, AUX2));

	// configure roll and throttle to be in hold mode
	failsafeChannelConfigs(AUX1)->mode = RX_FAILSAFE_MODE_HOLD;
	failsafeChannelConfigs(AUX2)->mode = RX_FAILSAFE_MODE_HOLD;

	EXPECT_TRUE(rx_is_healthy(&rx));

	// disconnect receiver
	for(int c = 0; c < 6; c++){
		mock_rc_pwm[c] = 0;
	}

	rx_run(&rx, MAX_INVALID_PULSE_TIME);

	// check that rx has gone into failsafe
	EXPECT_FALSE(rx_is_healthy(&rx));

	// should have hold
	EXPECT_EQ(1600, rx_get_channel(&rx, AUX1));
	EXPECT_EQ(1600, rx_get_channel(&rx, AUX2));

	failsafeChannelConfigs(AUX1)->mode = RX_FAILSAFE_MODE_SET;
	failsafeChannelConfigs(AUX2)->mode = RX_FAILSAFE_MODE_SET;

	failsafeChannelConfigs(AUX1)->step = 13;
	failsafeChannelConfigs(AUX2)->step = 13;

	rx_run(&rx, MAX_INVALID_PULSE_TIME);

	// should have hold
	EXPECT_EQ(RXFAIL_STEP_TO_CHANNEL_VALUE(13), rx_get_channel(&rx, AUX1));
	EXPECT_EQ(RXFAIL_STEP_TO_CHANNEL_VALUE(13), rx_get_channel(&rx, AUX2));

	failsafeChannelConfigs(AUX1)->mode = RX_FAILSAFE_MODE_AUTO;
	failsafeChannelConfigs(AUX2)->mode = RX_FAILSAFE_MODE_AUTO;

	rx_run(&rx, MAX_INVALID_PULSE_TIME);

	EXPECT_EQ(rxConfig()->midrc, rx_get_channel(&rx, AUX1));
	EXPECT_EQ(rxConfig()->midrc, rx_get_channel(&rx, AUX2));
}
// STUBS

extern "C" {
/*
    bool rcModeIsActive(boxId_e modeId) { return rcModeActivationMask & (1 << modeId); }

    void failsafeOnValidDataFailed() {}
    void failsafeOnValidDataReceived() {}

    void failsafeOnRxSuspend(uint32_t ) {}
    void failsafeOnRxResume(void) {}

    uint32_t micros(void) { return 0; }
    uint32_t millis(void) { return 0; }

    bool feature(uint32_t mask) {
        UNUSED(mask);
        return false;
    }

    bool isPPMDataBeingReceived(void) {
        return testData.isPPMDataBeingReceived;
    }

    bool isPWMDataBeingReceived(void) {
        return testData.isPWMDataBeingReceived;
    }

    void resetPPMDataReceivedState(void) {}

    bool rxMspFrameComplete(void) { return false; }

    void rxMspInit(rxRuntimeConfig_t *, rcReadRawDataPtr *) {}

    void rxPwmInit(rxRuntimeConfig_t *, rcReadRawDataPtr *) {}
	*/
}

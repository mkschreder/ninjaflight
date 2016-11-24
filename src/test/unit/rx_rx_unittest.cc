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

    uint32_t rcModeActivationMask;

    bool isPulseValid(uint16_t pulseDuration);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

#define DE_ACTIVATE_ALL_BOXES   0

typedef struct testData_s {
    bool isPPMDataBeingReceived;
    bool isPWMDataBeingReceived;
} testData_t;

static testData_t testData;

TEST(RxTest, TestValidFlightChannels)
{
    // given
    memset(&testData, 0, sizeof(testData));
    rcModeActivationMask = DE_ACTIVATE_ALL_BOXES;   // BOXFAILSAFE must be OFF

    // and
    modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];

    memset(rxConfig(), 0, sizeof(*rxConfig()));
    rxConfig()->rx_min_usec = 1000;
    rxConfig()->rx_max_usec = 2000;

    memset(&modeActivationConditions, 0, sizeof(modeActivationConditions));
    modeActivationConditions[0].auxChannelIndex = 0;
    modeActivationConditions[0].modeId = BOXARM;
    modeActivationConditions[0].range.startStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MIN);
    modeActivationConditions[0].range.endStep = CHANNEL_VALUE_TO_STEP(1600);

    // when
	struct rx rx;
	struct failsafe failsafe;
	failsafe_init(&failsafe, &rx, mock_syscalls());
    rx_init(&rx, mock_syscalls(), &failsafe, modeActivationConditions);

    // then (ARM channel should be positioned just 1 step above active range to init to OFF)
    EXPECT_EQ(1625, rx_get_channel(&rx, modeActivationConditions[0].auxChannelIndex +  NON_AUX_CHANNEL_COUNT));

    // given
    rx_flight_chans_reset(&rx);

    // and
    for (uint8_t channelIndex = 0; channelIndex < MAX_SUPPORTED_RC_CHANNEL_COUNT; channelIndex++) {
        bool validPulse = isPulseValid(1500);
        rx_flight_chans_update(&rx, channelIndex, validPulse);
    }

    // then
    EXPECT_TRUE(rx_flight_chans_valid(&rx));
}

TEST(RxTest, TestInvalidFlightChannels)
{
    // given
    memset(&testData, 0, sizeof(testData));

    // and
    modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];

    memset(rxConfig(), 0, sizeof(*rxConfig()));
    rxConfig()->rx_min_usec = 1000;
    rxConfig()->rx_max_usec = 2000;

    memset(&modeActivationConditions, 0, sizeof(modeActivationConditions));
    modeActivationConditions[0].auxChannelIndex = 0;
    modeActivationConditions[0].modeId = BOXARM;
    modeActivationConditions[0].range.startStep = CHANNEL_VALUE_TO_STEP(1400);
    modeActivationConditions[0].range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);

    // and
    uint16_t channelPulses[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    memset(&channelPulses, 1500, sizeof(channelPulses));

    // and
	struct rx rx;
	struct failsafe failsafe;
	failsafe_init(&failsafe, &rx, mock_syscalls());
    rx_init(&rx, mock_syscalls(), &failsafe, modeActivationConditions);

    // then (ARM channel should be positioned just 1 step below active range to init to OFF)
    EXPECT_EQ(1375, rx_get_channel(&rx, modeActivationConditions[0].auxChannelIndex +  NON_AUX_CHANNEL_COUNT));

    // and
    for (uint8_t stickChannelIndex = 0; stickChannelIndex < STICK_CHANNEL_COUNT; stickChannelIndex++) {

        // given
        rx_flight_chans_reset(&rx);

        for (uint8_t otherStickChannelIndex = 0; otherStickChannelIndex < STICK_CHANNEL_COUNT; otherStickChannelIndex++) {
            channelPulses[otherStickChannelIndex] = rxConfig()->rx_min_usec;
        }
        channelPulses[stickChannelIndex] = rxConfig()->rx_min_usec - 1;

        // when
        for (uint8_t channelIndex = 0; channelIndex < MAX_SUPPORTED_RC_CHANNEL_COUNT; channelIndex++) {
            bool validPulse = isPulseValid(channelPulses[channelIndex]);
            rx_flight_chans_update(&rx, channelIndex, validPulse);
        }

        // then
        EXPECT_FALSE(rx_flight_chans_valid(&rx));

        // given
        rx_flight_chans_reset(&rx);

        for (uint8_t otherStickChannelIndex = 0; otherStickChannelIndex < STICK_CHANNEL_COUNT; otherStickChannelIndex++) {
            channelPulses[otherStickChannelIndex] = rxConfig()->rx_max_usec;
        }
        channelPulses[stickChannelIndex] = rxConfig()->rx_max_usec + 1;

        // when
        for (uint8_t channelIndex = 0; channelIndex < MAX_SUPPORTED_RC_CHANNEL_COUNT; channelIndex++) {
            bool validPulse = isPulseValid(channelPulses[channelIndex]);
            rx_flight_chans_update(&rx, channelIndex, validPulse);
        }

        // then
        EXPECT_FALSE(rx_flight_chans_valid(&rx));
    }
}


// STUBS

extern "C" {
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
}

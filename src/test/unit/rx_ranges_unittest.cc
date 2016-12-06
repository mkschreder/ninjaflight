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

    #include "rx/rx.h"
    #include "common/maths.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

#define DE_ACTIVATE_ALL_BOXES   0

extern "C" {
    uint32_t rcModeActivationMask;
}

#if 0
// TODO: rx ranges
#define RANGE_CONFIGURATION(min, max) (const rxChannelRangeConfiguration_t) {min, max}

uint16_t testApplyRxChannelRangeConfiguraton(int sample, rxChannelRangeConfiguration_t range) {
    return applyRxChannelRangeConfiguraton(sample, &range);
}

TEST(RxChannelRangeTest, TestRxChannelRanges)
{
    rcModeActivationMask = DE_ACTIVATE_ALL_BOXES;   // BOXFAILSAFE must be OFF

    // No signal, special condition
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(0, RANGE_CONFIGURATION(1000, 2000)), 0);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(0, RANGE_CONFIGURATION(1300, 1700)), 0);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(0, RANGE_CONFIGURATION(900, 2100)), 0);

    // Exact mapping
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(1000, RANGE_CONFIGURATION(1000, 2000)), 1000);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(1500, RANGE_CONFIGURATION(1000, 2000)), 1500);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(2000, RANGE_CONFIGURATION(1000, 2000)), 2000);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(700, RANGE_CONFIGURATION(1000, 2000)), 750);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(2500, RANGE_CONFIGURATION(1000, 2000)), 2250);

    // Reversed channel
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(1000, RANGE_CONFIGURATION(2000, 1000)), 2000);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(1500, RANGE_CONFIGURATION(2000, 1000)), 1500);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(2000, RANGE_CONFIGURATION(2000, 1000)), 1000);

    // Shifted range
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(900, RANGE_CONFIGURATION(900, 1900)), 1000);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(1400, RANGE_CONFIGURATION(900, 1900)), 1500);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(1900, RANGE_CONFIGURATION(900, 1900)), 2000);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(600, RANGE_CONFIGURATION(900, 1900)), 750);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(2500, RANGE_CONFIGURATION(900, 1900)), 2250);
    
    // Narrower range than expected
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(1300, RANGE_CONFIGURATION(1300, 1700)), 1000);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(1500, RANGE_CONFIGURATION(1300, 1700)), 1500);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(1700, RANGE_CONFIGURATION(1300, 1700)), 2000);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(700, RANGE_CONFIGURATION(1300, 1700)), 750);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(2500, RANGE_CONFIGURATION(1300, 1700)), 2250);

    // Wider range than expected
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(900, RANGE_CONFIGURATION(900, 2100)), 1000);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(1500, RANGE_CONFIGURATION(900, 2100)), 1500);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(2100, RANGE_CONFIGURATION(900, 2100)), 2000);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(600, RANGE_CONFIGURATION(900, 2100)), 750);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(2700, RANGE_CONFIGURATION(900, 2100)), 2250);
    
    // extreme out of range
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(1, RANGE_CONFIGURATION(1000, 2000)), 750);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(1, RANGE_CONFIGURATION(1300, 1700)), 750);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(1, RANGE_CONFIGURATION(900, 2100)), 750);

    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(10000, RANGE_CONFIGURATION(1000, 2000)), 2250);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(10000, RANGE_CONFIGURATION(1300, 1700)), 2250);
    EXPECT_EQ(testApplyRxChannelRangeConfiguraton(10000, RANGE_CONFIGURATION(900, 2100)), 2250);
}
#endif

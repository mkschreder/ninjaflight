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
    #include "debug.h"

    #include <platform.h>

    #include "build_config.h"

    #include "common/axis.h"
    #include "common/maths.h"

    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"

    #include "config/runtime_config.h"
    #include "config/config.h"

    #include "io/beeper.h"
    #include "io/rc_controls.h"

    #include "rx/rx.h"
    #include "flight/failsafe.h"

    PG_REGISTER_PROFILE(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);

	PG_REGISTER_WITH_RESET_TEMPLATE(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 0);

	PG_RESET_TEMPLATE(failsafeConfig_t, failsafeConfig, 10, 200, 1000, 100, 0, 0); 

    extern uint32_t rcModeActivationMask;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

uint32_t testFeatureMask = 0;
uint16_t flightModeFlags = 0;
uint16_t testMinThrottle = 0;
throttleStatus_e throttleStatus = THROTTLE_HIGH;

enum {
    COUNTER_MW_DISARM = 0,
};
#define CALL_COUNT_ITEM_COUNT 1

static int callCounts[CALL_COUNT_ITEM_COUNT];

#define CALL_COUNTER(item) (callCounts[item])

void resetCallCounters(void) {
    memset(&callCounts, 0, sizeof(callCounts));
}

#define TEST_MID_RC 1495            // something other than the default 1500 will suffice.
#define TEST_MIN_CHECK 1100;
#define PERIOD_OF_10_SCONDS 10000
#define DE_ACTIVATE_ALL_BOXES 0

uint32_t sysTickUptime;

void configureFailsafe(void)
{
    memset(rxConfig(), 0, sizeof(*rxConfig()));
    rxConfig()->midrc = TEST_MID_RC;
    rxConfig()->mincheck = TEST_MIN_CHECK;

    memset(failsafeConfig(), 0, sizeof(*failsafeConfig()));
    failsafeConfig()->failsafe_delay = 10; // 1 second
    failsafeConfig()->failsafe_off_delay = 50; // 5 seconds
    failsafeConfig()->failsafe_kill_switch = false;
    failsafeConfig()->failsafe_throttle = 1200;
    failsafeConfig()->failsafe_throttle_low_delay = 50; // 5 seconds
    sysTickUptime = 0;
}
//
// Stepwise tests
//

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeInitialState)
{
    // given
    configureFailsafe();
    // and
    DISABLE_ARMING_FLAG(ARMED);

    // when
    failsafeInit();

    // then
    EXPECT_EQ(false, failsafeIsMonitoring());
    EXPECT_EQ(false, failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeStartMonitoring)
{
    // when
    failsafeStartMonitoring();

    // then
    EXPECT_EQ(true, failsafeIsMonitoring());
    EXPECT_EQ(false, failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeFirstArmedCycle)
{
    // given
    ENABLE_ARMING_FLAG(ARMED);

    // when
    failsafeOnValidDataFailed();                    // set last invalid sample at current time
    sysTickUptime += PERIOD_RXDATA_RECOVERY + 1;    // adjust time to point just past the recovery time to
    failsafeOnValidDataReceived();                  // cause a recovered link

    // and
    failsafeUpdateState();

    // then
    EXPECT_EQ(false, failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeNotActivatedWhenReceivingData)
{
    // when
    for (sysTickUptime = 0; sysTickUptime < PERIOD_OF_10_SCONDS; sysTickUptime++) {
        failsafeOnValidDataReceived();

        failsafeUpdateState();

        // then
        EXPECT_EQ(false, failsafeIsActive());
        EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    }
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeDetectsRxLossAndStartsLanding)
{
    // given
    ENABLE_ARMING_FLAG(ARMED);

    // and
    failsafeStartMonitoring();
    throttleStatus = THROTTLE_HIGH;                 // throttle HIGH to go for a failsafe landing procedure
    sysTickUptime = 0;                              // restart time from 0
    failsafeOnValidDataReceived();                  // set last valid sample at current time

    // when
    for (sysTickUptime = 0; sysTickUptime < (uint32_t)(failsafeConfig()->failsafe_delay * MILLIS_PER_TENTH_SECOND + PERIOD_RXDATA_FAILURE); sysTickUptime++) {
        failsafeOnValidDataFailed();

        failsafeUpdateState();

        // then
        EXPECT_EQ(false, failsafeIsActive());
        EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    }

    // given
    sysTickUptime++;                                // adjust time to point just past the failure time to
    failsafeOnValidDataFailed();                    // cause a lost link

    // when
    failsafeUpdateState();

    // then
    EXPECT_EQ(FAILSAFE_LANDING, failsafePhase());
    EXPECT_EQ(true, failsafeIsActive());
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeCausesLanding)
{
    // given
    sysTickUptime += failsafeConfig()->failsafe_off_delay * MILLIS_PER_TENTH_SECOND;
    sysTickUptime++;

    // when
    // no call to failsafeOnValidDataReceived();
    failsafeUpdateState();

    // then
    EXPECT_EQ(true, failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(ARMING_FLAG(PREVENT_ARMING));

    // given
    failsafeOnValidDataFailed();                    // set last invalid sample at current time
    sysTickUptime += PERIOD_RXDATA_RECOVERY + 1;    // adjust time to point just past the recovery time to
    failsafeOnValidDataReceived();                  // cause a recovered link

    // when
    failsafeUpdateState();

    // then
    EXPECT_EQ(true, failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(ARMING_FLAG(PREVENT_ARMING));

    // given
    sysTickUptime += PERIOD_OF_30_SECONDS + 1;      // adjust time to point just past the required additional recovery time
    failsafeOnValidDataReceived();

    // when
    failsafeUpdateState();

    // then
    EXPECT_EQ(false, failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM)); // disarm not called repeatedly.
    EXPECT_FALSE(ARMING_FLAG(PREVENT_ARMING));
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeDetectsRxLossAndJustDisarms)
{
    // given
    DISABLE_ARMING_FLAG(ARMED);
    resetCallCounters();

    // and
    failsafeStartMonitoring();
    throttleStatus = THROTTLE_LOW;                  // throttle LOW to go for a failsafe just-disarm procedure
    sysTickUptime = 0;                              // restart time from 0
    failsafeOnValidDataReceived();                  // set last valid sample at current time

    // when
    for (sysTickUptime = 0; sysTickUptime < (uint32_t)(failsafeConfig()->failsafe_delay * MILLIS_PER_TENTH_SECOND + PERIOD_RXDATA_FAILURE); sysTickUptime++) {
        failsafeOnValidDataFailed();

        failsafeUpdateState();

        // then
        EXPECT_EQ(false, failsafeIsActive());
        EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    }

    // given
    sysTickUptime++;                                // adjust time to point just past the failure time to
    failsafeOnValidDataFailed();                    // cause a lost link
    ENABLE_ARMING_FLAG(ARMED);                      // armed from here (disarmed state has cleared throttleLowPeriod).

    // when
    failsafeUpdateState();

    // then
    EXPECT_EQ(true, failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(ARMING_FLAG(PREVENT_ARMING));

    // given
    failsafeOnValidDataFailed();                    // set last invalid sample at current time
    sysTickUptime += PERIOD_RXDATA_RECOVERY + 1;    // adjust time to point just past the recovery time to
    failsafeOnValidDataReceived();                  // cause a recovered link

    // when
    failsafeUpdateState();

    // then
    EXPECT_EQ(true, failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(ARMING_FLAG(PREVENT_ARMING));

    // given
    sysTickUptime += PERIOD_OF_3_SECONDS + 1;       // adjust time to point just past the required additional recovery time
    failsafeOnValidDataReceived();

    // when
    failsafeUpdateState();

    // then
    EXPECT_EQ(false, failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));  // disarm not called repeatedly.
    EXPECT_FALSE(ARMING_FLAG(PREVENT_ARMING));
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeDetectsKillswitchEvent)
{
    // given
    ENABLE_ARMING_FLAG(ARMED);
    resetCallCounters();
    failsafeStartMonitoring();

    // and
    throttleStatus = THROTTLE_HIGH;                 // throttle HIGH to go for a failsafe landing procedure
    failsafeConfig()->failsafe_kill_switch = 1;        // configure AUX switch as kill switch
    rcModeActivationMask |= (1 << BOXFAILSAFE);     // and activate it
    sysTickUptime = 0;                              // restart time from 0
    failsafeOnValidDataReceived();                  // set last valid sample at current time
    sysTickUptime = PERIOD_RXDATA_FAILURE + 1;      // adjust time to point just past the failure time to
    failsafeOnValidDataFailed();                    // cause a lost link

    // when
    failsafeUpdateState();                          // kill switch handling should come first

    // then
    EXPECT_EQ(true, failsafeIsActive());
    EXPECT_TRUE(ARMING_FLAG(PREVENT_ARMING));
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());

    // given
    failsafeOnValidDataFailed();                    // set last invalid sample at current time
    sysTickUptime += PERIOD_RXDATA_RECOVERY + 1;    // adjust time to point just past the recovery time to
    failsafeOnValidDataReceived();                  // cause a recovered link

    rcModeActivationMask = DE_ACTIVATE_ALL_BOXES;   // BOXFAILSAFE must be off (kill switch)

    // when
    failsafeUpdateState();

    // then
    EXPECT_EQ(true, failsafeIsActive());
    EXPECT_TRUE(ARMING_FLAG(PREVENT_ARMING));
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());

    // given
    sysTickUptime += PERIOD_OF_1_SECONDS + 1;       // adjust time to point just past the required additional recovery time
    failsafeOnValidDataReceived();

    // when
    failsafeUpdateState();

    // then
    EXPECT_EQ(false, failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));  // disarm not called repeatedly.
    EXPECT_FALSE(ARMING_FLAG(PREVENT_ARMING));
}

/****************************************************************************************/
//
// Additional non-stepwise tests
//
/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeNotActivatedWhenDisarmedAndRXLossIsDetected)
{
    // given
    configureFailsafe();

    // and
    failsafeInit();

    // and
    DISABLE_ARMING_FLAG(ARMED);

    // when
    failsafeStartMonitoring();

    // and
    sysTickUptime = 0;                              // restart time from 0
    failsafeOnValidDataReceived();                  // set last valid sample at current time

    // when
    for (sysTickUptime = 0; sysTickUptime < PERIOD_RXDATA_FAILURE; sysTickUptime++) {
        failsafeOnValidDataFailed();

        failsafeUpdateState();

        // then
        EXPECT_EQ(false, failsafeIsActive());
        EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    }

    // given
    sysTickUptime++;                                // adjust time to point just past the failure time to
    failsafeOnValidDataFailed();                    // cause a lost link

    // when
    failsafeUpdateState();

    // then
    EXPECT_EQ(true, failsafeIsMonitoring());
    EXPECT_EQ(false, failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_FALSE(ARMING_FLAG(PREVENT_ARMING));
}

// STUBS

extern "C" {
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
// TODO: proper way to do this is to write a mock receiver
int16_t rc_get_channel_value(uint8_t id){ return rcData[id]; }
void rc_set_channel_value(uint8_t id, int16_t value){ rcData[id] = value; }

uint8_t armingFlags;
int16_t rcCommand[4];
uint32_t rcModeActivationMask = 0;
int16_t debug[DEBUG16_VALUE_COUNT];
bool isUsingSticksToArm = true;

bool rcModeIsActive(boxId_e modeId) { return rcModeActivationMask & (1 << modeId); }

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}

throttleStatus_e calculateThrottleStatus(rxConfig_t *rxConfig, uint16_t deadband3d_throttle)
{
    UNUSED(rxConfig);
    UNUSED(deadband3d_throttle);
    return throttleStatus;
}

bool feature(uint32_t mask) {
    return (mask & testFeatureMask);
}

void mwDisarm(void) {
    callCounts[COUNTER_MW_DISARM]++;
}

void beeper(beeperMode_e mode) {
    UNUSED(mode);
}

uint16_t enableFlightMode(flightModeFlags_e mask)
{
    flightModeFlags |= (mask);
    return flightModeFlags;
}

uint16_t disableFlightMode(flightModeFlags_e mask)
{
    flightModeFlags &= ~(mask);
    return flightModeFlags;
}

uint16_t getCurrentMinthrottle(void)
{
    return testMinThrottle;
}

bool isUsingSticksForArming(void)
{
    return isUsingSticksToArm;
}

}

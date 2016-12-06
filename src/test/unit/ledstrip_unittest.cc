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
#include <stdlib.h>

#include <limits.h>

//#define DEBUG_LEDSTRIP

// TODO: ledstrip
#if 0
extern "C" {
    #include "build_config.h"

    #include "common/color.h"
    #include "common/axis.h"
    #include "common/utils.h"

    #include "config/config.h"

    #include "rx/rx.h"
	#include "flight/failsafe.h"

    #include "sensors/battery.h"

    #include "drivers/light_ws2811strip.h"
    #include "io/ledstrip.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    extern uint8_t highestYValueForNorth;
    extern uint8_t lowestYValueForSouth;
    extern uint8_t highestXValueForWest;
    extern uint8_t lowestXValueForEast;
    extern uint8_t ledGridWidth;
    extern uint8_t ledGridHeight;

    void determineLedStripDimensions(void);
    void determineOrientationLimits(void);
}

// utility macros
#define LF(name) LED_FLAG_FUNCTION(LED_FUNCTION_ ## name)
#define LD(name) LED_FLAG_DIRECTION(LED_DIRECTION_ ## name)

class LedStripTest : public ::testing::Test {
protected:
    virtual void SetUp() {
		mock_system_reset();

		config_reset(&config);

		rx_init(&rx, mock_syscalls(), &config);
		rx_set_type(&rx, RX_PPM);
		rx_config_set_mapping(&config.rx, "AERT1234");

		ledstrip_init(&ledstrip, &config, mock_syscalls(), &rx, &failsafe);
    }
	struct ledstrip ledstrip;
	struct config config;
	struct rx rx;
	struct failsafe failsafe;
};


TEST_F(LedStripTest, parseLedStripConfig)
{
    // given
    static const struct led_config expectedLedStripConfig[WS2811_LED_STRIP_LENGTH] = {
            { CALCULATE_LED_XY( 9,  9), 0, LD(SOUTH) | LF(FLIGHT_MODE) | LF(WARNING) },
            { CALCULATE_LED_XY(10, 10), 0, LD(SOUTH) | LF(FLIGHT_MODE) | LF(WARNING) },
            { CALCULATE_LED_XY(11, 11), 0, LD(SOUTH) | LF(INDICATOR) | LF(ARM_STATE) },
            { CALCULATE_LED_XY(11, 11), 0, LD(EAST)  | LF(INDICATOR) | LF(ARM_STATE) },
            { CALCULATE_LED_XY(10, 10), 0, LD(EAST)  | LF(FLIGHT_MODE) },

            { CALCULATE_LED_XY(10,  5), 0, LD(SOUTH) | LF(FLIGHT_MODE) },
            { CALCULATE_LED_XY(11,  4), 0, LD(SOUTH) | LF(FLIGHT_MODE) },
            { CALCULATE_LED_XY(12,  3), 0, LD(SOUTH) | LF(INDICATOR) | LF(ARM_STATE) },
            { CALCULATE_LED_XY(12,  2), 0, LD(NORTH) | LF(INDICATOR) | LF(ARM_STATE) },
            { CALCULATE_LED_XY(11,  1), 0, LD(NORTH) | LF(FLIGHT_MODE) },
            { CALCULATE_LED_XY(10,  0), 0, LD(NORTH) | LF(FLIGHT_MODE) },

            { CALCULATE_LED_XY( 7,  0), 0, LD(NORTH) | LF(FLIGHT_MODE) | LF(WARNING) },
            { CALCULATE_LED_XY( 6,  0), 1, LD(NORTH) | LF(COLOR) | LF(WARNING) },
            { CALCULATE_LED_XY( 5,  0), 1, LD(NORTH) | LF(COLOR) | LF(WARNING) },
            { CALCULATE_LED_XY( 4,  0), 0, LD(NORTH) | LF(FLIGHT_MODE) | LF(WARNING) },

            { CALCULATE_LED_XY( 2,  0), 0, LD(NORTH) | LF(FLIGHT_MODE) },
            { CALCULATE_LED_XY( 1,  1), 0, LD(NORTH) | LF(FLIGHT_MODE) },
            { CALCULATE_LED_XY( 0,  2), 0, LD(NORTH) | LF(INDICATOR) | LF(ARM_STATE) },
            { CALCULATE_LED_XY( 0,  3), 0, LD(WEST)  | LF(INDICATOR) | LF(ARM_STATE) },
            { CALCULATE_LED_XY( 1,  4), 0, LD(WEST)  | LF(FLIGHT_MODE) },
            { CALCULATE_LED_XY( 2,  5), 0, LD(WEST)  | LF(FLIGHT_MODE) },

            { CALCULATE_LED_XY( 1, 10), 0, LD(WEST)  | LF(FLIGHT_MODE) },
            { CALCULATE_LED_XY( 0, 11), 0, LD(WEST)  | LF(INDICATOR) | LF(ARM_STATE) },
            { CALCULATE_LED_XY( 0, 11), 0, LD(SOUTH) | LF(INDICATOR) | LF(ARM_STATE) },
            { CALCULATE_LED_XY( 1, 10), 0, LD(SOUTH) | LF(FLIGHT_MODE) | LF(WARNING) },
            { CALCULATE_LED_XY( 2,  9), 0, LD(SOUTH) | LF(FLIGHT_MODE) | LF(WARNING) },

            { CALCULATE_LED_XY( 7,  7), 14, LF(THRUST_RING) },
            { CALCULATE_LED_XY( 8,  7), 15, LF(THRUST_RING) },
            { CALCULATE_LED_XY( 8,  8), 14, LF(THRUST_RING) },
            { CALCULATE_LED_XY( 7,  8), 15, LF(THRUST_RING) },

            { 0, 0, 0 },
            { 0, 0, 0 },
    };

    // and
    const char *ledStripConfigCommands[] = {
            // Spider quad

            // right rear cluster
            "9,9:S:FW:0",
            "10,10:S:FW:0",
            "11,11:S:IA:0",
            "11,11:E:IA:0",
            "10,10:E:F:0",

            // right front cluster
            "10,5:S:F:0",
            "11,4:S:F:0",
            "12,3:S:IA:0",
            "12,2:N:IA:0",
            "11,1:N:F:0",
            "10,0:N:F:0",

            // center front cluster
            "7,0:N:FW:0",
            "6,0:N:CW:1",
            "5,0:N:CW:1",
            "4,0:N:FW:0",

            // left front cluster
            "2,0:N:F:0",
            "1,1:N:F:0",
            "0,2:N:IA:0",
            "0,3:W:IA:0",
            "1,4:W:F:0",
            "2,5:W:F:0",

            // left rear cluster
            "1,10:W:F:0",
            "0,11:W:IA:0",
            "0,11:S:IA:0",
            "1,10:S:FW:0",
            "2,9:S:FW:0",

            // thrust ring
            "7,7::R:14",
            "8,7::R:15",
            "8,8::R:14",
            "7,8::R:15"
    };
    // and
	    // and
    bool result = true;

    // when
    for (unsigned index = 0; index < ARRAYLEN(ledStripConfigCommands); index++) {
        result = result && ledstrip_config_set_color(&config.ledstrip, index, ledStripConfigCommands[index]);
    }

	ledstrip_init(&ledstrip, &config, mock_syscalls(), &rx, &failsafe);

    // then
    EXPECT_TRUE(result);
    EXPECT_EQ(30, ledstrip.ledCount);
    EXPECT_EQ(4, ledstrip.ledRingCount);


    // and
    for (int index = 0; index < WS2811_LED_STRIP_LENGTH; index++) {
#ifdef DEBUG_LEDSTRIP
        printf("iteration: %d\n", index);
#endif
        EXPECT_EQ(expectedLedStripConfig[index].xy, config.ledstrip.leds[index].xy);
        EXPECT_EQ(expectedLedStripConfig[index].flags, config.ledstrip.leds[index].flags);
        EXPECT_EQ(expectedLedStripConfig[index].color, config.ledstrip.leds[index].color);
    }

    // then
    EXPECT_EQ(13, ledstrip.ledGridWidth);
    EXPECT_EQ(12, ledstrip.ledGridHeight);

    // then
    EXPECT_EQ(5, ledstrip.highestXValueForWest);
    EXPECT_EQ(7, ledstrip.lowestXValueForEast);
    EXPECT_EQ(5, ledstrip.highestYValueForNorth);
    EXPECT_EQ(6, ledstrip.lowestYValueForSouth);
}

TEST_F(LedStripTest, smallestGridWithCenter)
{
    // and
    static const struct led_config testLedConfigs[] = {
        { CALCULATE_LED_XY( 2,  2), 0, LD(SOUTH) | LD(EAST) | LF(INDICATOR) | LF(ARM_STATE) },
        { CALCULATE_LED_XY( 2,  1), 0, LD(EAST)             | LF(FLIGHT_MODE) | LF(WARNING) },
        { CALCULATE_LED_XY( 2,  0), 0, LD(NORTH) | LD(EAST) | LF(INDICATOR) | LF(ARM_STATE) },
        { CALCULATE_LED_XY( 1,  0), 0, LD(NORTH)            | LF(FLIGHT_MODE) | LF(WARNING) },
        { CALCULATE_LED_XY( 0,  0), 0, LD(NORTH) | LD(WEST) | LF(INDICATOR) | LF(ARM_STATE) },
        { CALCULATE_LED_XY( 0,  1), 0, LD(WEST)             | LF(FLIGHT_MODE) | LF(WARNING) },
        { CALCULATE_LED_XY( 0,  2), 0, LD(SOUTH) | LD(WEST) | LF(INDICATOR) | LF(ARM_STATE) },
        { CALCULATE_LED_XY( 1,  2), 0, LD(SOUTH)            | LF(FLIGHT_MODE) | LF(WARNING) }
    };
    memcpy(config.ledstrip.leds, &testLedConfigs, sizeof(testLedConfigs));

    // when
	ledstrip_reload_config(&ledstrip);

    // then
    EXPECT_EQ(3, ledstrip.ledGridWidth);
    EXPECT_EQ(3, ledstrip.ledGridHeight);

    // when
	ledstrip_reload_config(&ledstrip);

    // then
    EXPECT_EQ(0, ledstrip.highestXValueForWest);
    EXPECT_EQ(2, ledstrip.lowestXValueForEast);
    EXPECT_EQ(0, ledstrip.highestYValueForNorth);
    EXPECT_EQ(2, ledstrip.lowestYValueForSouth);
}

TEST_F(LedStripTest, smallestGrid)
{
    // and
    static const struct led_config testLedConfigs[] = {
        { CALCULATE_LED_XY( 1,  1), 0, LD(SOUTH) | LD(EAST) | LF(INDICATOR) | LF(FLIGHT_MODE) },
        { CALCULATE_LED_XY( 1,  0), 0, LD(NORTH) | LD(EAST) | LF(INDICATOR) | LF(FLIGHT_MODE) },
        { CALCULATE_LED_XY( 0,  0), 0, LD(NORTH) | LD(WEST) | LF(INDICATOR) | LF(FLIGHT_MODE) },
        { CALCULATE_LED_XY( 0,  1), 0, LD(SOUTH) | LD(WEST) | LF(INDICATOR) | LF(FLIGHT_MODE) },
    };
    memcpy(config.ledstrip.leds, &testLedConfigs, sizeof(testLedConfigs));
	
    // when
	ledstrip_reload_config(&ledstrip);

    // then
    EXPECT_EQ(2, ledstrip.ledGridWidth);
    EXPECT_EQ(2, ledstrip.ledGridHeight);

    // when
	ledstrip_reload_config(&ledstrip);

    // then
    EXPECT_EQ(0, ledstrip.highestXValueForWest);
    EXPECT_EQ(1, ledstrip.lowestXValueForEast);
    EXPECT_EQ(0, ledstrip.highestYValueForNorth);
    EXPECT_EQ(1, ledstrip.lowestYValueForSouth);
}

/*
        { CALCULATE_LED_XY( 1, 14), 0, LD(SOUTH) | LF(FLIGHT_MODE) | LF(INDICATOR) | LF(FLIGHT_MODE) },

        { CALCULATE_LED_XY( 0, 13), 0, LD(WEST)  | LF(INDICATOR) | LF(ARM_STATE) },
        { CALCULATE_LED_XY( 0, 12), 0, LD(WEST)  | LF(INDICATOR) | LF(ARM_STATE) },

        { CALCULATE_LED_XY( 0, 11), 0, LD(WEST)  | LF(FLIGHT_MODE) },
        { CALCULATE_LED_XY( 0, 10), 0, LD(WEST)  | LF(FLIGHT_MODE) },
        { CALCULATE_LED_XY( 0,  9), 0, LD(WEST)  | LF(FLIGHT_MODE) },
        { CALCULATE_LED_XY( 0,  8), 0, LD(WEST)  | LF(FLIGHT_MODE) | LF(WARNING) },
        { CALCULATE_LED_XY( 0,  7), 0, LD(WEST)  | LF(FLIGHT_MODE) | LF(WARNING) },
        { CALCULATE_LED_XY( 0,  6), 0, LD(WEST)  | LF(FLIGHT_MODE) | LF(WARNING) },
        { CALCULATE_LED_XY( 0,  5), 0, LD(WEST)  | LF(FLIGHT_MODE) },
        { CALCULATE_LED_XY( 0,  4), 0, LD(WEST)  | LF(FLIGHT_MODE) },
        { CALCULATE_LED_XY( 0,  3), 0, LD(WEST)  | LF(FLIGHT_MODE) },

        { CALCULATE_LED_XY( 0,  2), 0, LD(WEST)  | LF(INDICATOR) | LF(ARM_STATE) },
        { CALCULATE_LED_XY( 0,  1), 0, LD(WEST)  | LF(INDICATOR) | LF(ARM_STATE) },

        { CALCULATE_LED_XY( 1,  0), 0, LD(NORTH) | LF(FLIGHT_MODE) | LF(INDICATOR) | LF(ARM_STATE) },
        { CALCULATE_LED_XY( 2,  0), 0, LD(NORTH) | LF(FLIGHT_MODE) | LF(ARM_STATE) },
        { CALCULATE_LED_XY( 3,  0), 0, LD(NORTH) | LF(FLIGHT_MODE) | LF(INDICATOR) | LF(ARM_STATE) },

        { CALCULATE_LED_XY( 4,  1), 0, LD(EAST)  | LF(INDICATOR) | LF(ARM_STATE) },
        { CALCULATE_LED_XY( 4,  2), 0, LD(EAST)  | LF(INDICATOR) | LF(ARM_STATE) },

        { CALCULATE_LED_XY( 4,  3), 0, LD(EAST)  | LF(FLIGHT_MODE) },
        { CALCULATE_LED_XY( 4,  4), 0, LD(EAST)  | LF(FLIGHT_MODE) },
        { CALCULATE_LED_XY( 4,  5), 0, LD(EAST)  | LF(FLIGHT_MODE) },
        { CALCULATE_LED_XY( 4,  6), 0, LD(EAST)  | LF(FLIGHT_MODE) | LF(WARNING) },
        { CALCULATE_LED_XY( 4,  7), 0, LD(EAST)  | LF(FLIGHT_MODE) | LF(WARNING) },
        { CALCULATE_LED_XY( 4,  8), 0, LD(EAST)  | LF(FLIGHT_MODE) | LF(WARNING) },
        { CALCULATE_LED_XY( 4,  9), 0, LD(EAST)  | LF(FLIGHT_MODE) },
        { CALCULATE_LED_XY( 4, 10), 0, LD(EAST)  | LF(FLIGHT_MODE) },
        { CALCULATE_LED_XY( 4, 11), 0, LD(EAST)  | LF(FLIGHT_MODE) },

        { CALCULATE_LED_XY( 4, 12), 0, LD(EAST)  | LF(INDICATOR) | LF(ARM_STATE) },
        { CALCULATE_LED_XY( 4, 13), 0, LD(EAST)  | LF(INDICATOR) | LF(ARM_STATE) },

        { CALCULATE_LED_XY( 3, 14), 0, LD(SOUTH) | LF(FLIGHT_MODE) | LF(INDICATOR) | LF(ARM_STATE) },

 */

hsvColor_t testColors[LED_CONFIGURABLE_COLOR_COUNT];

#define TEST_COLOR_COUNT 4

TEST_F(LedStripTest, parseColor)
{
    // and
    const hsvColor_t expectedColors[TEST_COLOR_COUNT] = {
            //  H    S    V
            {   0,   0,   0 },
            {   1,   1,   1 },
            { 359, 255, 255 },
            { 333,  22,   1 }
    };

    const char *testColors[TEST_COLOR_COUNT] = {
            "0,0,0",
            "1,1,1",
            "359,255,255",
            "333,22,1"
    };

    // when
    for (int index = 0; index < TEST_COLOR_COUNT; index++) {
#ifdef DEBUG_LEDSTRIP
        printf("parse iteration: %d\n", index);
#endif

        ledstrip_config_set_color(&config.ledstrip, index, testColors[index]);
    }

    // then

    for (int index = 0; index < TEST_COLOR_COUNT; index++) {
#ifdef DEBUG_LEDSTRIP
        printf("iteration: %d\n", index);
#endif

        EXPECT_EQ(expectedColors[index].h, config.ledstrip.colors[index].h);
        EXPECT_EQ(expectedColors[index].s, config.ledstrip.colors[index].s);
        EXPECT_EQ(expectedColors[index].v, config.ledstrip.colors[index].v);
    }
}

extern "C" {

uint8_t armingFlags = 0;
uint16_t flightModeFlags = 0;

uint32_t rcModeActivationMask;

battery_state_t battery_get_state(struct battery *self) {
	UNUSED(self);
    return BATTERY_OK;
}

void ws2811LedStripInit(void) {}
void ws2811UpdateStrip(void) {}

void setLedValue(int index, const uint8_t value) {
    UNUSED(index);
    UNUSED(value);
}

void setLedHsv(int index, const hsvColor_t *color) {
    UNUSED(index);
    UNUSED(color);
}

void getLedHsv(int index, hsvColor_t *color) {
    UNUSED(index);
    UNUSED(color);
}


void scaleLedValue(int index, const uint8_t scalePercent) {
    UNUSED(index);
    UNUSED(scalePercent);
}

void setStripColor(const hsvColor_t *color) {
    UNUSED(color);
}

void setStripColors(const hsvColor_t *colors) {
    UNUSED(colors);
}

bool isWS2811LedStripReady(void) { return false; }

void tfp_sprintf(char *, char*, ...) { }

bool rcModeIsActive(boxId_e modeId) { return rcModeActivationMask & (1 << modeId); }
bool failsafeIsActive() { return false; }
bool rxIsReceivingSignal() { return true; }
    bool sensors(uint32_t mask) { UNUSED(mask); return true; }

uint8_t GPS_numSat;
uint8_t stateFlags;
uint16_t rssi;

}
#endif

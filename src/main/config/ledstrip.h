/*
 * This file is part of Ninjaflight.
 *
 * Copyright: collective.
 * Cleanup: Martin Schröder <mkschreder.uk@gmail.com>
 * Original source from Cleanflight.
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

#pragma once

#include "../common/color.h"

#define LED_SPECIAL_COLOR_COUNT         8
#define LED_DIRECTION_COUNT             6
#define LED_MAX_STRIP_LENGTH           32
#define LED_CONFIGURABLE_COLOR_COUNT   16
#define LED_MODE_COUNT                  6
#define LED_FUNCTION_COUNT             10

#define LED_DIRECTION_BIT_OFFSET 0
#define LED_FUNCTION_BIT_OFFSET LED_DIRECTION_COUNT

// These are used by configuration defaults
#define LED_X_BIT_OFFSET 4
#define LED_Y_BIT_OFFSET 0
#define LED_XY_MASK      0x0F
#define CALCULATE_LED_XY(x, y) ((((x) & LED_XY_MASK) << LED_X_BIT_OFFSET) | (((y) & LED_XY_MASK) << LED_Y_BIT_OFFSET))

typedef enum {
    LED_MODE_ORIENTATION = 0,
    LED_MODE_HEADFREE,
    LED_MODE_HORIZON,
    LED_MODE_ANGLE,
    LED_MODE_MAG,
    LED_MODE_BARO,
    LED_SPECIAL
} ledModeIndex_e;

typedef enum {
    LED_SCOLOR_DISARMED = 0,
    LED_SCOLOR_ARMED,
    LED_SCOLOR_ANIMATION,
    LED_SCOLOR_BACKGROUND,
    LED_SCOLOR_BLINKBACKGROUND,
    LED_SCOLOR_GPSNOSATS,
    LED_SCOLOR_GPSNOLOCK,
    LED_SCOLOR_GPSLOCKED
} ledSpecialColorIds_e;


// TODO: this should be named led_color_t or something
typedef enum {
    COLOR_BLACK = 0,
    COLOR_WHITE,
    COLOR_RED,
    COLOR_ORANGE,
    COLOR_YELLOW,
    COLOR_LIME_GREEN,
    COLOR_GREEN,
    COLOR_MINT_GREEN,
    COLOR_CYAN,
    COLOR_LIGHT_BLUE,
    COLOR_BLUE,
    COLOR_DARK_VIOLET,
    COLOR_MAGENTA,
    COLOR_DEEP_PINK,
} colorId_e;

typedef enum {
    LED_DIRECTION_NORTH = 0,
    LED_DIRECTION_EAST,
    LED_DIRECTION_SOUTH,
    LED_DIRECTION_WEST,
    LED_DIRECTION_UP,
    LED_DIRECTION_DOWN
} ledDirectionId_e;

#define LED_FLAG_DIRECTION(directionId) (1 << (LED_DIRECTION_BIT_OFFSET + (directionId)))
// generate direction bit, used in initializers
#define LED_FLAG_DIRECTION_MASK (((1 << LED_DIRECTION_COUNT) - 1) << LED_DIRECTION_BIT_OFFSET)

typedef enum {
    LED_FUNCTION_INDICATOR,
    LED_FUNCTION_WARNING,
    LED_FUNCTION_FLIGHT_MODE,
    LED_FUNCTION_ARM_STATE,
    LED_FUNCTION_THROTTLE,
    LED_FUNCTION_THRUST_RING,
    LED_FUNCTION_COLOR,
    LED_FUNCTION_GPS,
    LED_FUNCTION_RSSI,
    LED_FUNCTION_BLINK,
} ledFunctionId_e;

#define LED_FLAG_FUNCTION(functionId) (1 << (LED_FUNCTION_BIT_OFFSET + (functionId)))
// generate direction bit, used in initializers
#define LED_FLAG_FUNCTION_MASK (((1 << LED_FUNCTION_COUNT) - 1) << LED_FUNCTION_BIT_OFFSET)

struct led_spc_color_indices {
    uint8_t color[LED_SPECIAL_COLOR_COUNT];
};

struct led_mode_color_indices {
    uint8_t color[LED_DIRECTION_COUNT];
};

struct led_config {
    uint8_t xy;     // see LED_X/Y_MASK defines
    uint8_t color;  // see colors (config_master)
    uint16_t flags; // see LED_FLAG_FUNCTION + LED_FLAG_DIRECTION
};

struct ledstrip_config {
	struct led_config leds[LED_MAX_STRIP_LENGTH];
	struct hsvColor_s colors[LED_CONFIGURABLE_COLOR_COUNT];
	struct led_mode_color_indices modeColors[LED_MODE_COUNT];
	struct led_spc_color_indices spcColors[1];
};

bool ledstrip_config_set_color(struct ledstrip_config *self, int index, const char *colorConfig);

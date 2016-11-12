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

typedef struct specialColorIndexes_s {
    uint8_t color[LED_SPECIAL_COLOR_COUNT];
} specialColorIndexes_t;

typedef struct modeColorIndexes_s {
    uint8_t color[LED_DIRECTION_COUNT];
} modeColorIndexes_t;

typedef struct ledConfig_s {
    uint8_t xy;     // see LED_X/Y_MASK defines
    uint8_t color;  // see colors (config_master)
    uint16_t flags; // see LED_FLAG_FUNCTION + LED_FLAG_DIRECTION
} ledConfig_t;

PG_DECLARE_ARR(ledConfig_t, LED_MAX_STRIP_LENGTH, ledConfigs);
PG_DECLARE_ARR(hsvColor_t, LED_CONFIGURABLE_COLOR_COUNT, colors);
PG_DECLARE_ARR(modeColorIndexes_t, LED_MODE_COUNT, modeColors);
PG_DECLARE_ARR(specialColorIndexes_t, 1, specialColors);



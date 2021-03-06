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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>

#include <platform.h>

#include <build_config.h>

#include <common/color.h>
#include <common/maths.h>
#include <common/typeconversion.h>
#include <common/printf.h>
#include <common/axis.h>
#include <common/utils.h>

#include "drivers/light_ws2811strip.h"
#include "drivers/system.h"
#include "drivers/serial.h"

#include "io/ledstrip.h"

#include "config/config.h"
#include "config/feature.h"

#include "rx/rx.h"

//#define USE_LED_ANIMATION
//#define USE_LED_RING_DEFAULT_CONFIG

#define LED_STRIP_HZ(hz) ((int32_t)((1000 * 1000) / (hz)))
#define LED_STRIP_MS(ms) ((int32_t)(1000 * (ms)))

#if LED_MAX_STRIP_LENGTH > WS2811_LED_STRIP_LENGTH
# error "Led strip length must match driver"
#endif

// macro to save typing on default colors
#define HSV(color) (hsv[COLOR_ ## color])

/*
 * 6 coords @nn,nn
 * 4 direction @##
 * 6 modes @####
 * = 16 bytes per led
 * 16 * 32 leds = 512 bytes storage needed worst case.
 * = not efficient to store led configs as strings in flash.
 * = becomes a problem to send all the data via cli due to serial/cli buffers
 */



static void updateLedRingCounts(struct ledstrip *self);

static void determineLedStripDimensions(struct ledstrip *self){
	int maxX = 0;
	int maxY = 0;

	for (int ledIndex = 0; ledIndex < self->ledCount; ledIndex++) {
		const struct led_config *ledConfig = &self->config->ledstrip.leds[ledIndex];

		maxX = MAX(ledGetX(ledConfig), maxX);
		maxY = MAX(ledGetY(ledConfig), maxY);
	}
	self->ledGridWidth = maxX + 1;
	self->ledGridHeight = maxY + 1;
}

static void determineOrientationLimits(struct ledstrip *self){
	self->highestYValueForNorth = (self->ledGridHeight / 2) - 1;
	self->lowestYValueForSouth = ((self->ledGridHeight + 1) / 2);
	self->highestXValueForWest = (self->ledGridWidth / 2) - 1;
	self->lowestXValueForEast = ((self->ledGridWidth + 1) / 2);
}

static void updateLedCount(struct ledstrip *self){
	int count = 0, countRing = 0;

	for (int ledIndex = 0; ledIndex < LED_MAX_STRIP_LENGTH; ledIndex++) {
		const struct led_config *ledConfig = &self->config->ledstrip.leds[ledIndex];
		if (ledConfig->flags == 0 && ledConfig->xy == 0)
			break;
		count++;
		if ((ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_THRUST_RING)))
			countRing++;
	}
	self->ledCount = count;
	self->ledRingCount = countRing;
}

void ledstrip_reload_config(struct ledstrip *self){
	updateLedCount(self);
	determineLedStripDimensions(self);
	determineOrientationLimits(self);
	updateLedRingCounts(self);
}

// get specialColor by index
static const struct hsvColor_s *getSC(struct ledstrip *self, ledSpecialColorIds_e index){
	return &self->config->ledstrip.colors[self->config->ledstrip.spcColors[0].color[index]];
}

typedef enum {
	// the ordering is important, see below how NSEW is mapped to  NE/SE/NW/SW
	QUADRANT_NORTH	  = 1 << 0,
	QUADRANT_SOUTH	  = 1 << 1,
	QUADRANT_EAST	   = 1 << 2,
	QUADRANT_WEST	   = 1 << 3,
	QUADRANT_NORTH_EAST = 1 << 4,
	QUADRANT_SOUTH_EAST = 1 << 5,
	QUADRANT_NORTH_WEST = 1 << 6,
	QUADRANT_SOUTH_WEST = 1 << 7,
	QUADRANT_NONE	   = 1 << 8,
	QUADRANT_NOTDIAG	= 1 << 9,  // not in NE/SE/NW/SW
	// values for test
	QUADRANT_ANY		= QUADRANT_NORTH | QUADRANT_SOUTH | QUADRANT_EAST | QUADRANT_WEST | QUADRANT_NONE,
} quadrant_e;

static quadrant_e __attribute__((unused)) getLedQuadrant(struct ledstrip *self, const int ledIndex){
	const struct led_config *ledConfig = &self->config->ledstrip.leds[ledIndex];

	int quad = 0;
	if (ledGetY(ledConfig) <= self->highestYValueForNorth)
		quad |= QUADRANT_NORTH;
	else if (ledGetY(ledConfig) >= self->lowestYValueForSouth)
		quad |= QUADRANT_SOUTH;
	if (ledGetX(ledConfig) >= self->lowestXValueForEast)
		quad |= QUADRANT_EAST;
	else if (ledGetX(ledConfig) <= self->highestXValueForWest)
		quad |= QUADRANT_WEST;

	if((quad & (QUADRANT_NORTH | QUADRANT_SOUTH))
	   && (quad & (QUADRANT_EAST | QUADRANT_WEST)) ) { // is led  in one of NE/SE/NW/SW?
		quad |= 1 << (4 + ((quad & QUADRANT_SOUTH) ? 1 : 0) + ((quad & QUADRANT_WEST) ? 2 : 0));
	} else {
		quad |= QUADRANT_NOTDIAG;
	}

	if((quad & (QUADRANT_NORTH | QUADRANT_SOUTH | QUADRANT_EAST | QUADRANT_WEST))  == 0)
		quad |= QUADRANT_NONE;

	return quad;
}

static const struct {
	uint8_t dir;			 // ledDirectionId_e
	uint16_t quadrantMask;   // quadrant_e
} directionQuadrantMap[] = {
	{LED_DIRECTION_SOUTH, QUADRANT_SOUTH},
	{LED_DIRECTION_NORTH, QUADRANT_NORTH},
	{LED_DIRECTION_EAST,  QUADRANT_EAST},
	{LED_DIRECTION_WEST,  QUADRANT_WEST},
	{LED_DIRECTION_DOWN,  QUADRANT_ANY},
	{LED_DIRECTION_UP,	QUADRANT_ANY},
};
/*
static hsvColor_t * getDirectionalModeColor(struct ledstrip *self, const int ledIndex, const modeColorIndexes_t *modeColors){
	const ledConfig_t *ledConfig = ledConfigs(ledIndex);

	quadrant_e quad = getLedQuadrant(self, ledIndex);
	for(unsigned i = 0; i < ARRAYLEN(directionQuadrantMap); i++) {
		ledDirectionId_e dir = directionQuadrantMap[i].dir;
		quadrant_e quadMask = directionQuadrantMap[i].quadrantMask;

		if((ledConfig->flags & LED_FLAG_DIRECTION(dir))
		   && (quad & quadMask))
			return colors(modeColors->color[dir]);
	}
	return NULL;
}
*/

// map flight mode to led mode, in order of priority
// flightMode == 0 is always active
#if 0
static const struct {
	uint16_t flightMode;
	uint8_t ledMode;
} flightModeToLed[] = {
	{HEADFREE_MODE, LED_MODE_HEADFREE},
#if USE_MAG == 1
	{MAG_MODE,	  LED_MODE_MAG},
#endif
#ifdef BARO
	{BARO_MODE,	 LED_MODE_BARO},
#endif
	{HORIZON_MODE,  LED_MODE_HORIZON},
	{ANGLE_MODE,	LED_MODE_ANGLE},
	{0,			 LED_MODE_ORIENTATION},
};

static void applyLedModeLayer(struct ledstrip *self){
	for (int ledIndex = 0; ledIndex < self->ledCount; ledIndex++) {
		const ledConfig_t *ledConfig = ledConfigs(ledIndex);
		const hsvColor_t* color = NULL;

		if (!(ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_THRUST_RING))) {
			if (ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_COLOR)) {
				color = colors(ledConfig->color);
			} else {
				color = getSC(LED_SCOLOR_BACKGROUND);
			}
		}

		if (ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_FLIGHT_MODE)) {
			for(unsigned i = 0; i < ARRAYLEN(flightModeToLed); i++)
				if(!flightModeToLed[i].flightMode || FLIGHT_MODE(flightModeToLed[i].flightMode)) {
					color = getDirectionalModeColor(self, ledIndex, modeColors(flightModeToLed[i].ledMode));
					break; // stop on first match
				}
		} else if (ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_ARM_STATE)) {
			color = ARMING_FLAG(ARMED) ? getSC(LED_SCOLOR_ARMED) : getSC(LED_SCOLOR_DISARMED);
		}

		if(color)
			setLedHsv(ledIndex, color);
	}
}
#endif 

static void applyLedHue(struct ledstrip *self, ledFunctionId_e flag, int16_t value, int16_t minRange, int16_t maxRange){
	int scaled = scaleRange(value, minRange, maxRange, -60, +60);
	scaled += HSV_HUE_MAX;   // wrap negative values correctly

	for (int i = 0; i < self->ledCount; ++i) {
		const struct led_config *ledConfig = &self->config->ledstrip.leds[i];
		if (!(ledConfig->flags & LED_FLAG_FUNCTION(flag)))
			continue;

		hsvColor_t color;
		getLedHsv(i, &color);
		color.h = (color.h + scaled) % (HSV_HUE_MAX + 1);
		setLedHsv(i, &color);
	}
}

static void applyLedHueLayer(struct ledstrip *self)
{
	applyLedHue(self, LED_FUNCTION_THROTTLE, rx_get_channel(self->rx, THROTTLE), PWM_RANGE_MIN, PWM_RANGE_MAX);
	applyLedHue(self, LED_FUNCTION_RSSI, rx_get_rssi(self->rx), 0, 1023);
}

typedef enum {
	WARNING_ARMING_DISABLED,
	WARNING_LOW_BATTERY,
	WARNING_FAILSAFE,
} warningFlags_e;

const hsvColor_t hsv[] = {
	//						H	S	V
	[COLOR_BLACK] =		{  0,   0,   0},
	[COLOR_WHITE] =		{  0, 255, 255},
	[COLOR_RED] =		  {  0,   0, 255},
	[COLOR_ORANGE] =	   { 30,   0, 255},
	[COLOR_YELLOW] =	   { 60,   0, 255},
	[COLOR_LIME_GREEN] =   { 90,   0, 255},
	[COLOR_GREEN] =		{120,   0, 255},
	[COLOR_MINT_GREEN] =   {150,   0, 255},
	[COLOR_CYAN] =		 {180,   0, 255},
	[COLOR_LIGHT_BLUE] =   {210,   0, 255},
	[COLOR_BLUE] =		 {240,   0, 255},
	[COLOR_DARK_VIOLET] =  {270,   0, 255},
	[COLOR_MAGENTA] =	  {300,   0, 255},
	[COLOR_DEEP_PINK] =	{330,   0, 255},
};

static void applyLedWarningLayer(struct ledstrip *self, bool updateNow, uint32_t *timer){
	static uint8_t warningFlashCounter = 0;
	static uint8_t warningFlags = 0;		  // non-zero during blinks

	if (updateNow) {
		// keep counter running, so it stays in sync with blink
		warningFlashCounter++;
		if (warningFlashCounter >= 20) {
			warningFlashCounter = 0;
		}
		if (warningFlashCounter == 0) {	  // update when old flags was processed
			warningFlags = 0;
			// TODO: refactor. This type of battery logic should be outside of ledstrip module!
			/*
			if (feature(FEATURE_VBAT) && battery_get_state(self->battery) != BATTERY_OK)
				warningFlags |= 1 << WARNING_LOW_BATTERY;
			if (feature(FEATURE_FAILSAFE) && failsafe_is_active(self->failsafe))
				warningFlags |= 1 << WARNING_FAILSAFE;
			*/
		}
		*timer += LED_STRIP_HZ(10);
	}

	if (warningFlags) {
		const hsvColor_t *warningColor =  &HSV(BLACK);

		bool colorOn = (warningFlashCounter % 2) == 0;   // w_w_
		warningFlags_e warningId = warningFlashCounter / 4;
		if(warningFlags & (1 << warningId)) {
			switch(warningId) {
				case WARNING_ARMING_DISABLED:
					warningColor = colorOn ? &HSV(GREEN)  : &HSV(BLACK);
					break;
				case WARNING_LOW_BATTERY:
					warningColor = colorOn ? &HSV(RED)	: &HSV(BLACK);
					break;
				case WARNING_FAILSAFE:
					warningColor = colorOn ? &HSV(YELLOW) : &HSV(BLUE);
					break;
				default:;
			}
		}

		for (int ledIndex = 0; ledIndex < self->ledCount; ledIndex++) {
			const struct led_config *ledConfig = &self->config->ledstrip.leds[ledIndex];
			if (!(ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_WARNING)))
				continue;

			setLedHsv(ledIndex, warningColor);
		}
	}
}

#ifdef GPS
static void __attribute__((unused)) applyLedGpsLayer(struct ledstrip *self, bool updateNow, uint32_t *timer){
	(void)self;
	(void)updateNow;
	(void)timer;
/*
	static uint8_t gpsFlashCounter = 0;
	static uint8_t gpsPauseCounter = 0;
	const uint8_t blinkPauseLength = 4;

	if (updateNow) {
		if (gpsPauseCounter > 0) {
			gpsPauseCounter--;
		} else if (gpsFlashCounter >= GPS_numSat) {
			gpsFlashCounter = 0;
			gpsPauseCounter = blinkPauseLength;
		} else {
			gpsFlashCounter++;
			gpsPauseCounter = 1;
		}
		*timer += LED_STRIP_HZ(2.5);
	}

	const hsvColor_t *gpsColor;
	// TODO: ledstrip gps stuff
	if (GPS_numSat == 0 || !sensors(SENSOR_GPS)) {
		gpsColor = getSC(LED_SCOLOR_GPSNOSATS);
	} else {
		bool colorOn = gpsPauseCounter == 0;  // each interval starts with pause
		if(STATE(GPS_FIX)) {
			gpsColor = colorOn ? getSC(LED_SCOLOR_GPSLOCKED) : getSC(LED_SCOLOR_BACKGROUND);
		} else {
			gpsColor = colorOn ? getSC(LED_SCOLOR_GPSNOLOCK) : getSC(LED_SCOLOR_GPSNOSATS);
		}
	}
	for (int i = 0; i < self->ledCount; ++i) {
		const ledConfig_t *ledConfig = ledConfigs(i);
		if (!(ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_GPS)))
			continue;

		setLedHsv(i, gpsColor);
	}
*/
}
#endif

#define INDICATOR_DEADBAND 25

static void applyLedIndicatorLayer(struct ledstrip *self, bool updateNow, uint32_t *timer){
	static uint8_t flashCounter = 0;

	if(updateNow) {
		if (!rx_has_signal(self->rx)) {
			*timer += LED_STRIP_HZ(5);  // try again soon
		} else {
			// calculate update frequency
			int scale = 0; //MAX(ABS(rcCommand[ROLL]), ABS(rcCommand[PITCH]));  // 0 - 500
			scale += (50 - INDICATOR_DEADBAND);  // start increasing frequency right after deadband
			*timer += LED_STRIP_HZ(5) * 50 / MAX(50, scale);   // 5 - 50Hz update, 2.5 - 25Hz blink

			flashCounter = !flashCounter;
		}
	}
/*
	const hsvColor_t *flashColor = flashCounter ? &HSV(ORANGE) : &HSV(BLACK); // TODO - use user color?

	quadrant_e quadrants = 0;
	if (rcCommand[ROLL] > INDICATOR_DEADBAND) {
		quadrants |= QUADRANT_NORTH_EAST | QUADRANT_SOUTH_EAST;
	} else if (rcCommand[ROLL] < -INDICATOR_DEADBAND) {
		quadrants |= QUADRANT_NORTH_WEST | QUADRANT_SOUTH_WEST;
	}
	if (rcCommand[PITCH] > INDICATOR_DEADBAND) {
		quadrants |= QUADRANT_NORTH_EAST | QUADRANT_NORTH_WEST;
	} else if (rcCommand[PITCH] < -INDICATOR_DEADBAND) {
		quadrants |= QUADRANT_SOUTH_EAST | QUADRANT_SOUTH_WEST;
	}

	for (int ledIndex = 0; ledIndex < self->ledCount; ledIndex++) {
		const ledConfig_t *ledConfig = ledConfigs(ledIndex);
		if (!(ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_INDICATOR)))
			continue;

		if(getLedQuadrant(self, ledIndex) & quadrants)
			setLedHsv(ledIndex, flashColor);
	}
	*/
}

#define ROTATION_SEQUENCE_LED_COUNT 6 // 2 on, 4 off
#define ROTATION_SEQUENCE_LED_WIDTH 2 // 2 on

static void updateLedRingCounts(struct ledstrip *self){
	int seqLen;
	// try to split in segments/rings of exactly ROTATION_SEQUENCE_LED_COUNT leds
	if ((self->ledRingCount % ROTATION_SEQUENCE_LED_COUNT) == 0) {
		seqLen = ROTATION_SEQUENCE_LED_COUNT;
	} else {
		seqLen = self->ledRingCount;
		// else split up in equal segments/rings of at most ROTATION_SEQUENCE_LED_COUNT leds
		// TODO - improve partitioning (15 leds -> 3x5)
		while ((seqLen > ROTATION_SEQUENCE_LED_COUNT) && ((seqLen % 2) == 0)) {
			seqLen /= 2;
		}
	}
	self->ledRingSeqLen = seqLen;
}

static void applyLedThrustRingLayer(struct ledstrip *self, bool updateNow, uint32_t *timer)
{
	static uint8_t rotationPhase;
	int ledRingIndex = 0;

	if(updateNow) {
		rotationPhase = rotationPhase > 0 ? rotationPhase - 1 : self->ledRingSeqLen - 1;

		int scale = scaleRange(rx_get_channel(self->rx, THROTTLE), PWM_RANGE_MIN, PWM_RANGE_MAX, 10, 100);
		*timer += LED_STRIP_HZ(5) * 10 / scale;  // 5 - 50Hz update rate
	}

	for (int ledIndex = 0; ledIndex < self->ledCount; ledIndex++) {
		const struct led_config *ledConfig = &self->config->ledstrip.leds[ledIndex];
		if (!(ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_THRUST_RING)))
			continue;

		bool applyColor;
		applyColor = (ledRingIndex + rotationPhase) % self->ledRingSeqLen < ROTATION_SEQUENCE_LED_WIDTH;

		const hsvColor_t *ringColor = applyColor ? &self->config->ledstrip.colors[ledConfig->color] : &HSV(BLACK);
		setLedHsv(ledIndex, ringColor);

		ledRingIndex++;
	}
}

// blink twice, then wait
static void applyLedBlinkLayer(struct ledstrip *self, bool updateNow, uint32_t *timer)
{
	static uint8_t blinkCounter = 0;
	const int blinkCycleLength = 20;

	if (updateNow) {
		blinkCounter++;
		if (blinkCounter >= blinkCycleLength) {
			blinkCounter = 0;
		}
		*timer += LED_STRIP_HZ(10);
	}

	bool ledOn = (blinkCounter & 1) == 0 && blinkCounter < 4;  // b_b_____...

	for (int i = 0; i < self->ledCount; ++i) {
		const struct led_config *ledConfig = &self->config->ledstrip.leds[i];
		if (!(ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_BLINK)))
			continue;

		const hsvColor_t *blinkColor = ledOn ? &self->config->ledstrip.colors[ledConfig->color] : getSC(self, LED_SCOLOR_BLINKBACKGROUND);
		setLedHsv(i, blinkColor);
	}
}


#ifdef USE_LED_ANIMATION

static void applyLedAnimationLayer(bool updateNow, uint32_t *timer)
{
	static uint8_t frameCounter = 0;
	const int animationFrames = ledGridHeight;
	if(updateNow) {
		frameCounter = (frameCounter + 1 < animationFrames) ? frameCounter + 1 : 0;
		*timer += LED_STRIP_HZ(20);
	}

	if (ARMING_FLAG(ARMED))
		return;

	int previousRow = frameCounter > 0 ? frameCounter - 1 : animationFrames - 1;
	int currentRow = frameCounter;
	int nextRow = (frameCounter + 1 < animationFrames) ? frameCounter + 1 : 0;

	for (int ledIndex = 0; ledIndex < ledCount; ledIndex++) {
		const struct led_config *ledConfig = &self->config->ledstrip.leds[ledIndex];

		if (ledGetY(ledConfig) == previousRow) {
			setLedHsv(ledIndex, getSC(LED_SCOLOR_ANIMATION));
			scaleLedValue(ledIndex, 50);
		} else if (ledGetY(ledConfig) == currentRow) {
			setLedHsv(ledIndex, getSC(LED_SCOLOR_ANIMATION));
		} else if (ledGetY(ledConfig) == nextRow) {
			scaleLedValue(ledIndex, 50);
		}
	}
}
#endif

typedef enum {
	timIndicator,
	timBlink,
	timWarning,
#ifdef GPS
	timGps,
#endif
	timRotation,
#ifdef USE_LED_ANIMATION
	timAnimation,
#endif
	timTimerCount
} timId_e;

static uint32_t timerVal[timTimerCount];

// function to apply layer.
// function must replan self using timer pointer
// when updateNow is true (timer triggered), state must be updated first,
//  before calculating led state. Otherwise update started by different trigger
//  may modify LED state.
typedef void applyLayerFn_timed(struct ledstrip *self, bool updateNow, uint32_t* timer);
typedef void applyLayerFn(struct ledstrip *self);

static const struct {
	int8_t timId;						  // timer id for update, -1 if none
	union {
		applyLayerFn *apply;			   // function to apply layer unconditionally
		applyLayerFn_timed *applyTimed;	// apply with timer
	} f;
} layerTable[] = {
	// LAYER 1
	{ -1,			 .f.apply	  = &applyLedHueLayer },
	// LAYER 2
	{timWarning,	  .f.applyTimed = &applyLedWarningLayer},
#ifdef GPS
	{timGps,		  .f.applyTimed = &applyLedGpsLayer},
#endif
	// LAYER 3
	{timIndicator,	.f.applyTimed = &applyLedIndicatorLayer},
	// LAYER 4
	{timBlink,		.f.applyTimed = &applyLedBlinkLayer},
#ifdef USE_LED_ANIMATION
	{timAnimation,	.f.applyTimed = &applyLedAnimationLayer},
#endif
	{timRotation,	 .f.applyTimed = &applyLedThrustRingLayer},
};

void ledstrip_update(struct ledstrip *self){

	if (!(self->ledStripInitialised && isWS2811LedStripReady())) {
		return;
	}

	if(!self->ledStripEnabled) return;

	uint32_t now = sys_micros(self->system);

	// test all led timers, setting corresponding bits
	uint32_t timActive = 0;
	for(timId_e timId = 0; timId < timTimerCount; timId++) {
		if(cmp32(now, timerVal[timId]) < 0)
			continue;  // not ready yet
		timActive |= 1 << timId;
		// sanitize timer value, so that it can be safely incremented. Handles inital timerVal value.
		// max delay is limited to 5s
		if(cmp32(now, timerVal[timId]) >= LED_STRIP_MS(100) || cmp32(now, timerVal[timId]) < LED_STRIP_HZ(5000) ) {
			timerVal[timId] = now;
		}
	}

	if (!timActive)
		return;		  // no change this update, keep old state

	// apply all layers; triggered timed functions has to update timers
	for(unsigned i = 0; i < ARRAYLEN(layerTable); i++) {
		int timId = layerTable[i].timId;
		if(timId >= 0) {
			uint32_t *timer = &timerVal[timId];
			bool updateNow = timActive & (1 << timId);
			(*layerTable[i].f.applyTimed)(self, updateNow, timer);
		} else {
			(*layerTable[i].f.apply)(self);
		}
	}

	ws2811UpdateStrip();
}

void ledstrip_init(struct ledstrip *self, const struct config const *config, const struct system_calls *system, struct rx *rx, struct failsafe *failsafe){
	memset(self, 0, sizeof(struct ledstrip));
	self->rx = rx;
	self->system = system;
	self->failsafe = failsafe;
	self->config = config;
	self->ledStripInitialised = false;
	self->ledStripEnabled = true;
}

void ledstrip_enable(struct ledstrip *self){
	ledstrip_reload_config(self);
	self->ledStripInitialised = true;

	ws2811LedStripInit();
	self->ledStripEnabled = true;
}

void ledstrip_disable(struct ledstrip *self){
	if(!self->ledStripEnabled) return;
	setStripColor(&HSV(BLACK));

	ws2811UpdateStrip();
	self->ledStripEnabled = false;
}

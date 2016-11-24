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

#pragma once

#include <stddef.h>

#include "../config/ledstrip.h"

static inline int ledGetX(const ledConfig_t *lcfg) { return (lcfg->xy >> LED_X_BIT_OFFSET) & LED_XY_MASK; }
static inline int ledGetY(const ledConfig_t *lcfg) { return (lcfg->xy >> LED_Y_BIT_OFFSET) & LED_XY_MASK; }
static inline void ledSetXY(ledConfig_t *lcfg, int x, int y) {
    lcfg->xy = ((x & LED_XY_MASK) << LED_X_BIT_OFFSET) | ((y & LED_XY_MASK) << LED_Y_BIT_OFFSET);
}

struct ledstrip {
	bool ledStripInitialised;
	bool ledStripEnabled;

	uint8_t ledGridWidth;
	uint8_t ledGridHeight;
	// grid offsets
	uint8_t highestYValueForNorth;
	uint8_t lowestYValueForSouth;
	uint8_t highestXValueForWest;
	uint8_t lowestXValueForEast;

	uint8_t ledCount;
	uint8_t ledRingCount;
	uint8_t ledRingSeqLen;

	struct failsafe *failsafe;
	struct battery *battery;
	struct rx *rx;
};

void ledstrip_init(struct ledstrip *self, struct battery *bat, struct failsafe *failsafe, struct rx *rx);

bool ledstrip_set_led_config(struct ledstrip *self, int ledIndex, const char *config);
void ledstrip_update(struct ledstrip *self);
void ledstrip_update_ring(struct ledstrip *self);

void ledstrip_set_default_config(struct ledstrip *self);
void ledstrip_genconfig(struct ledstrip *self, int ledIndex, char *ledConfigBuffer, size_t bufferSize);

bool parseColor(int index, const char *colorConfig);
void applyDefaultColors(void);

void ledStripEnable(void);

//! Currently reloads config from the global store TODO: pass config to init!
void ledstrip_reload_config(struct ledstrip *self);

bool ledstrip_set_mode_color(struct ledstrip *self, ledModeIndex_e modeIndex, int modeColorIndex, int colorIndex);


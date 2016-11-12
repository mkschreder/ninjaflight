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

extern uint8_t ledCount;
extern uint8_t ledRingCount;
void ledStripInit(void);

bool parseLedStripConfig(int ledIndex, const char *config);
void updateLedStrip(void);
void updateLedRing(void);

void applyDefaultLedStripConfig(void);
void generateLedConfig(int ledIndex, char *ledConfigBuffer, size_t bufferSize);

bool parseColor(int index, const char *colorConfig);
void applyDefaultColors(void);

void ledStripInit(void);
void ledStripEnable(void);
void reevalulateLedConfig(void);

bool setModeColor(ledModeIndex_e modeIndex, int modeColorIndex, int colorIndex);


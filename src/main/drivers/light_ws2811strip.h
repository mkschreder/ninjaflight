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

#define WS2811_LED_STRIP_LENGTH 32
#define WS2811_BITS_PER_LED 24
#define WS2811_DELAY_BUFFER_LENGTH 42 // for 50us delay

#define WS2811_DATA_BUFFER_SIZE (WS2811_BITS_PER_LED * WS2811_LED_STRIP_LENGTH)

#define WS2811_DMA_BUFFER_SIZE (WS2811_DATA_BUFFER_SIZE + WS2811_DELAY_BUFFER_LENGTH)   // number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes)

#define BIT_COMPARE_1 17 // timer compare value for logical 1
#define BIT_COMPARE_0 9  // timer compare value for logical 0

void ws2811LedStripInit(void);

void ws2811LedStripHardwareInit(void);
void ws2811LedStripDMAEnable(void);

void ws2811UpdateStrip(void);

void setLedHsv(int index, const hsvColor_t *color);
void getLedHsv(int index, hsvColor_t *color);

void scaleLedValue(int index, const uint8_t scalePercent);
void setLedValue(int index, const uint8_t value);

void setStripColor(const hsvColor_t *color);
void setStripColors(const hsvColor_t *colors);

bool isWS2811LedStripReady(void);

extern uint8_t ledStripDMABuffer[WS2811_DMA_BUFFER_SIZE];
extern volatile uint8_t ws2811LedDataTransferInProgress;

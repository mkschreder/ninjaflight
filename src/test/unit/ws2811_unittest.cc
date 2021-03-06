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

extern "C" {
    #include <platform.h>

    #include "build_config.h"

    #include "common/color.h"

    #include "drivers/dma.h"
    #include "drivers/light_ws2811strip.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
extern uint16_t dmaBufferOffset;

void fastUpdateLEDDMABuffer(uint8_t **dst, rgbColor24bpp_t color);
}

TEST(WS2812, updateDMABuffer) {
    // given
    rgbColor24bpp_t color1 = { .raw = {0xFF,0xAA,0x55} };

    // and
    uint8_t *dst = ledStripDMABuffer;

    // when
    fastUpdateLEDDMABuffer(&dst, color1);

    // then
    EXPECT_EQ(24, dst - ledStripDMABuffer);

    // and
    uint8_t byteIndex = 0;

    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 0]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 1]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 2]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 3]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 4]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 5]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 6]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 7]);
    byteIndex++;

    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 0]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 1]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 2]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 3]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 4]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 5]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 6]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 7]);
    byteIndex++;

    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 0]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 1]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 2]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 3]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 4]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 5]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 6]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 7]);
    byteIndex++;
}

extern "C" {
rgbColor24bpp_t hsvToRgb24(const hsvColor_t *c) {
    UNUSED(c);
    return (rgbColor24bpp_t){.raw = {0,0,0}};
}

void ws2811LedStripHardwareInit(void) {}
void ws2811LedStripDMAEnable(void) {}

void dmaSetHandler(dmaHandlerIdentifier_e, dmaCallbackHandlerFuncPtr ) {}

uint8_t DMA_GetFlagStatus(uint32_t) { return 0; }
void DMA_Cmd(DMA_Channel_TypeDef*, FunctionalState ) {}
void DMA_ClearFlag(uint32_t) {}

}

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
#include <stdint.h>

#include <platform.h>

#include "build_config.h"

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/msp.h"

static uint16_t mspFrame[RX_MAX_SUPPORTED_RC_CHANNELS];
static bool rxMspFrameDone = false;

static uint16_t rxMspReadRawRC(rxRuntimeConfig_t *rxRuntimeConfigPtr, uint8_t chan)
{
    UNUSED(rxRuntimeConfigPtr);
    return mspFrame[chan];
}

void rxMspFrameReceive(uint16_t *frame, int channelCount)
{
    for (int i = 0; i < channelCount; i++) {
        mspFrame[i] = frame[i];
    }

    // Any channels not provided will be reset to zero
    for (int i = channelCount; i < RX_MAX_SUPPORTED_RC_CHANNELS; i++) {
        mspFrame[i] = 0;
    }

    rxMspFrameDone = true;
}

bool rxMspFrameComplete(void)
{
    if (!rxMspFrameDone) {
        return false;
    }

    rxMspFrameDone = false;
    return true;
}

bool rxMspInit(rxRuntimeConfig_t *rconf, rcReadRawDataPtr *callback){
    rconf->channelCount = RX_MAX_SUPPORTED_RC_CHANNELS;
    if (callback)
        *callback = rxMspReadRawRC;
	return true;
}

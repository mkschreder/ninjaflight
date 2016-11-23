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
#include <stdlib.h>

#include <string.h>

#include "build_config.h"

#include <platform.h>

#include "config/config.h"
#include "config/parameter_group.h"

#include "drivers/pwm_rx.h"

#include "config/config.h"
#include "config/parameter_group.h"
#include "config/feature.h"

#include "rx/rx.h"
#include "rx/pwm.h"

static const struct system_calls_pwm *syspwm = 0;

static uint16_t pwmReadRawRC(rxRuntimeConfig_t *rxRuntimeConfigPtr, uint8_t channel)
{
    UNUSED(rxRuntimeConfigPtr);
	if(!syspwm) return 1000;
	return syspwm->read_pwm(syspwm, channel);
}

static uint16_t ppmReadRawRC(rxRuntimeConfig_t *rxRuntimeConfigPtr, uint8_t channel)
{
    UNUSED(rxRuntimeConfigPtr);
	if(!syspwm) return 1000;
	return syspwm->read_ppm(syspwm, channel);
}

void rxPwmInit(const struct system_calls_pwm *pwm, rxRuntimeConfig_t *rxRuntimeConfigPtr, rcReadRawDataPtr *callback)
{
    UNUSED(rxRuntimeConfigPtr);
	syspwm = pwm;
    // configure PWM/CPPM read function and max number of channels. serial rx below will override both of these, if enabled
    if (feature(FEATURE_RX_PARALLEL_PWM)) {
        rxRuntimeConfigPtr->channelCount = MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT;
        *callback = pwmReadRawRC;
    }
    if (feature(FEATURE_RX_PPM)) {
        rxRuntimeConfigPtr->channelCount = MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT;
        *callback = ppmReadRawRC;
    }
}


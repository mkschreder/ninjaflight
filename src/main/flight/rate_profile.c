/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include <platform.h>

#include "build_config.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "io/rc_curves.h"

#include "flight/rate_profile.h"

static uint8_t currentControlRateProfileIndex = 0;
struct rate_config *currentControlRateProfile;

uint8_t getCurrentControlRateProfile(void)
{
    return currentControlRateProfileIndex;
}

struct rate_config *getControlRateConfig(uint8_t profileIndex)
{
    return controlRateProfiles(profileIndex);
}

void setControlRateProfile(uint8_t profileIndex)
{
    if (profileIndex >= MAX_CONTROL_RATE_PROFILE_COUNT) {
        profileIndex = 0;
    }
    currentControlRateProfileIndex = profileIndex;
    currentControlRateProfile = controlRateProfiles(profileIndex);
}

void activateControlRateConfig(void)
{
    generatePitchRollCurve();
    generateYawCurve();
    generateThrottleCurve();
}

void changeControlRateProfile(uint8_t profileIndex)
{
    setControlRateProfile(profileIndex);
    activateControlRateConfig();
}


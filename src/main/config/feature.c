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

#include "config/feature.h"

#if 0
bool config_feature_enabled(const struct config const *self, uint32_t mask){
    return config->feature.enabledFeatures & mask;
}

void featureSet(uint32_t mask)
{
    featureConfig()->enabledFeatures |= mask;
}

void featureClear(uint32_t mask)
{
    featureConfig()->enabledFeatures &= ~(mask);
}

void featureClearAll(void)
{
    featureConfig()->enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return featureConfig()->enabledFeatures;
}
#endif

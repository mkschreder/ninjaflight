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

struct rate_config {
    uint8_t rcRate8;
    uint8_t rcExpo8;
    uint8_t thrMid8;
    uint8_t thrExpo8;
    uint8_t rates[3];
    uint8_t dynThrPID;
    uint8_t rcYawExpo8;
    uint16_t tpa_breakpoint;                // Breakpoint at which TPA is activated
};

extern struct rate_config *currentControlRateProfile;

typedef struct rateProfileSelection_s {
    uint8_t defaultRateProfileIndex;
} rateProfileSelection_t;


void setControlRateProfile(uint8_t profileIndex);
uint8_t getCurrentControlRateProfile(void);
struct rate_config *getControlRateConfig(uint8_t profileIndex);
void resetControlRateConfig(struct rate_config *controlRateConfig);

void activateControlRateConfig(void);

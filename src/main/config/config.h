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

#define MAX_PROFILE_COUNT 3
#define ONESHOT_FEATURE_CHANGED_DELAY_ON_BOOT_MS 1500

#include "parameter_group.h"

void handleOneshotFeatureChangeOnRestart(void);

struct ninja;

void initEEPROM(void);
void ninja_config_reset(struct ninja *self);
void ninja_config_load(struct ninja *self);
void ninja_config_save(struct ninja *self);
void ninja_config_validate(struct ninja *self);
void ninja_config_save_and_beep(struct ninja *self);

void ninja_config_change_profile(struct ninja *self, uint8_t profileIndex);

void changeControlRateProfile(uint8_t profileIndex);

bool canSoftwareSerialBeUsed(void);
void configureRateProfileSelection(uint8_t profileIndex, uint8_t rateProfileIndex);

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

#include "common/filter.h"
#include "../io/rc_controls.h" // TODO: remove this because battery should not depend on throttleStatus_e
#include "../config/battery.h"

#define VBAT_SCALE_DEFAULT 110
#define VBAT_RESDIVVAL_DEFAULT 10
#define VBAT_RESDIVMULTIPLIER_DEFAULT 1
#define VBAT_SCALE_MIN 0
#define VBAT_SCALE_MAX 255

typedef enum {
    BATTERY_OK = 0,
    BATTERY_WARNING,
    BATTERY_CRITICAL,
    BATTERY_NOT_PRESENT
} battery_state_t;

struct battery {
	uint16_t vbat;
	uint16_t vbatRaw;
	uint16_t vbatLatestADC;
	uint8_t batteryCellCount;
	uint16_t batteryWarningVoltage;
	uint16_t batteryCriticalVoltage;
	uint16_t amperageLatestADC;
	int32_t amperage;
	int32_t mAhDrawn;

	battery_state_t batteryState;
	biquad_t vbatFilterState;

	struct battery_config *config;
};

// TODO: remove after done refactoring
extern struct battery default_battery;

void battery_init(struct battery *self, struct battery_config *config);
battery_state_t battery_get_state(struct battery *self);
const char *battery_get_state_str(struct battery *self);
void battery_update(struct battery *self);
uint8_t battery_get_remaining_percent(struct battery *self);
uint8_t battery_get_remaining_capacity(struct battery *self);
uint16_t battery_get_voltage(struct battery *self);
uint8_t battery_get_cell_count(struct battery *self);
uint16_t battery_get_cell_voltage(struct battery *self);
int32_t battery_get_current(struct battery *self);
int32_t battery_get_spent_capacity(struct battery *self);

void battery_update_current_meter(struct battery *self, int32_t lastUpdateAt, throttleStatus_e throttleStatus);

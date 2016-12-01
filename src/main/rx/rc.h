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

#include "../config/rx.h"
#include "../config/rc_controls.h"
#include "rc_command.h"
#include <stdbool.h>

struct rc {
	struct rc_command rc_command;
	uint32_t boxes;
	uint32_t keys;
	struct rx *rx;
};

typedef enum {
    THROTTLE_LOW = 0,
    THROTTLE_HIGH
} throttleStatus_e;

typedef enum {
    NOT_CENTERED = 0,
    CENTERED
} rollPitchStatus_e;

#define ROL_LO (1 << (2 * ROLL))
#define ROL_CE (3 << (2 * ROLL))
#define ROL_HI (2 << (2 * ROLL))
#define PIT_LO (1 << (2 * PITCH))
#define PIT_CE (3 << (2 * PITCH))
#define PIT_HI (2 << (2 * PITCH))
#define YAW_LO (1 << (2 * YAW))
#define YAW_CE (3 << (2 * YAW))
#define YAW_HI (2 << (2 * YAW))
#define THR_LO (1 << (2 * THROTTLE))
#define THR_CE (3 << (2 * THROTTLE))
#define THR_HI (2 << (2 * THROTTLE))

#define CHANNEL_RANGE_MIN 900
#define CHANNEL_RANGE_MAX 2100

#define MODE_STEP_TO_CHANNEL_VALUE(step) (CHANNEL_RANGE_MIN + 25 * (step))
#define CHANNEL_VALUE_TO_STEP(channelValue) ((constrain((channelValue), CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX) - CHANNEL_RANGE_MIN) / 25)

#define MIN_MODE_RANGE_STEP 0
#define MAX_MODE_RANGE_STEP ((CHANNEL_RANGE_MAX - CHANNEL_RANGE_MIN) / 25)

// Roll/pitch rates are a proportion used for mixing, so it tops out at 1.0:
#define CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX  100

/* Meaningful yaw rates are effectively unbounded because they are treated as a rotation rate multiplier: */
#define CONTROL_RATE_CONFIG_YAW_RATE_MAX         255

#define CONTROL_RATE_CONFIG_TPA_MAX              100

#define IS_RANGE_USABLE(range) ((range)->startStep < (range)->endStep)

struct rx;

// MUTUALLY EXCLUSIVE KEYS (only one active at a time)
#define RC_KEY_GYROCAL					(1UL << 1)
#define RC_KEY_ACCCAL					(1UL << 2)
#define RC_KEY_MAGCAL					(1UL << 3)
#define RC_KEY_PROFILE1					(1UL << 4)
#define RC_KEY_PROFILE2					(1UL << 5)
#define RC_KEY_PROFILE3					(1UL << 6)
#define RC_KEY_SAVE						(1UL << 7)
#define RC_KEY_ARM						(1UL << 8)
#define RC_KEY_DISARM					(1UL << 9)
#define RC_KEY_ACC_TRIM_PITCH_INC		(1UL << 10)
#define RC_KEY_ACC_TRIM_PITCH_DEC		(1UL << 11)
#define RC_KEY_ACC_TRIM_ROLL_INC		(1UL << 12)
#define RC_KEY_ACC_TRIM_ROLL_DEC		(1UL << 13)
#define RC_KEY_DISPLAY_OFF				(1UL << 14)
#define RC_KEY_DISPLAY_ON				(1UL << 15)
// NONEXCLUSIVE INPUTS (several can be active at the same time)
#define RC_KEY_LEVEL					(1UL << (16 + 0))
#define RC_KEY_BLEND					(1UL << (16 + 1))
#define RC_KEY_ALTSTAB					(1UL << (16 + 2))
#define RC_KEY_HEADSTAB					(1UL << (16 + 3))
#define RC_KEY_HEADFIX					(1UL << (16 + 4))
#define RC_KEY_CALIBRATE				(1UL << (16 + 5))

typedef uint64_t rc_key_t;

void rc_init(struct rc *self, struct rx *rx);
bool rc_key_active(struct rc *rc, rc_key_t key);
int16_t rc_get_command(struct rc *self, uint8_t axis);
void rc_update(struct rc *self);

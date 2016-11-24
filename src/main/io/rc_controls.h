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

typedef enum rc_alias {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8
} rc_alias_e;

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
extern int16_t rcCommand[4];
bool areUsingSticksToArm(void);

bool areSticksInApModePosition(uint16_t ap_mode);
throttleStatus_e calculateThrottleStatus(struct rx *self, rxConfig_t *rxConfig, uint16_t deadband3d_throttle);
rollPitchStatus_e calculateRollPitchCenterStatus(struct rx *self, rxConfig_t *rxConfig);

bool rcModeIsActive(boxId_e modeId);
void rcModeUpdateActivated(struct rx *self, modeActivationCondition_t *modeActivationConditions);
bool rcModeIsActivationConditionPresent(modeActivationCondition_t *modeActivationConditions, boxId_e modeId);


bool isUsingSticksForArming(void);

int32_t getRcStickDeflection(struct rx *self, int32_t axis, uint16_t midrc);

void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions);

bool isRangeActive(struct rx *self, uint8_t auxChannelIndex, channelRange_t *range);

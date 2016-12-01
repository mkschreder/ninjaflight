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

/**
 * Defines supported virtual keys. Certain stick combinations may trigger these
 * keys. Most of these keys specify a kind of virtual rc keyboard. How they are
 * actually interpreted is up to the current state of the flight controller.
 */
typedef enum {
	// these keys are result of stick movement
	RC_KEY_IDLE,
	RC_KEY_GYROCAL,
	RC_KEY_ACCCAL,
	RC_KEY_MAGCAL,
	RC_KEY_ACC_INFLIGHT_CALIB,
	RC_KEY_PROFILE1,
	RC_KEY_PROFILE2,
	RC_KEY_PROFILE3,
	RC_KEY_SAVE,
	RC_KEY_STICK_ARM,
	RC_KEY_STICK_DISARM,
	RC_KEY_ACC_TRIM_PITCH_INC,
	RC_KEY_ACC_TRIM_PITCH_DEC,
	RC_KEY_ACC_TRIM_ROLL_INC,
	RC_KEY_ACC_TRIM_ROLL_DEC,
	RC_KEY_DISPLAY_OFF,
	RC_KEY_DISPLAY_ON,
	// following keys are related to range functions and do not generate repeat events
	RC_KEY_FUNC_LEVEL,
	RC_KEY_FUNC_ARM,			//!< arming through box range
	RC_KEY_FUNC_BLEND,
	RC_KEY_FUNC_ALTSTAB,
	RC_KEY_FUNC_HEADSTAB,
	RC_KEY_FUNC_HEADFIX,
	RC_KEY_FUNC_CALIBRATE,
	RC_NUM_KEYS
} rc_key_t;

/**
 * This enum defines key state. To support cases where an rc range is not
 * configured (ie you can't really tell if the key is pressed or released) the
 * third option has been added which is returned by rc_key_state() if the key
 * does not exist.
 */
typedef enum {
	RC_KEY_DISABLED, //!< returned by rc_key_state() if the key does not exist (not configured etc.)
	RC_KEY_PRESSED, //!< returned by rc_key_state() when the key is pressed
	RC_KEY_RELEASED //!< returned by rc_key_state() when the key is not pressed
} rc_key_state_t;

/**
 * This event listener should be embedded in another struct and you can then
 * use container_of() macro to retreive pointer to your enclosing struct in the
 * event handler. Callbacks can be set to NULL if user does not wish to use them.
 */
struct rc_event_listener {
	/**
	 * This callback is called when key changes state.
	 */
	void (*on_key_state)(struct rc_event_listener *self, rc_key_t key, rc_key_state_t newstate);
	/**
	 * This callback is called when a stick combination (or a key) is held in
	 * PRESSED state for a duration of RC_KEY_REPEAT_MS. It is not called for
	 * range keys that are always held down.
	 */
	void (*on_key_repeat)(struct rc_event_listener *self, rc_key_t key, rc_key_state_t newstate);
};

struct rc {
	struct rc_command rc_command;
	uint32_t boxes;
	struct rx *rx;
	struct rc_event_listener *evl;
	//! holds the key bits. This mask grows as we add more keys.
	uint8_t key_mask[RC_NUM_KEYS / 8 + 1];
	uint8_t key_enabled_mask[RC_NUM_KEYS / 8 + 1];
	uint32_t range_mask;
};

typedef enum {
    THROTTLE_LOW = 0,
    THROTTLE_HIGH
} throttleStatus_e;

typedef enum {
    NOT_CENTERED = 0,
    CENTERED
} rollPitchStatus_e;

#define RC_KEY_REPEAT_MS 200

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
/**
 * Initializes a new rc struct.
 * @param rx instance of the underlying rx to read values from.
 * @param evl event listener or NULL
 */
void rc_init(struct rc *self, struct rx *rx, struct rc_event_listener *evl);

/**
 * @param key the key for which to return state
 * @return current state of the key (PRESSED, RELEASED, DISABLED)
 */
rc_key_state_t rc_key_state(struct rc *rc, rc_key_t key);

/**
 * This function is used to get stick commands after exponential curves have
 * been applied to the rx input.
 *
 * @param axis one of ROLL, PITCH, YAW, THROTTLE
 * @return a value between -500 and 500.
 */
int16_t rc_get_command(struct rc *self, uint8_t axis);

/**
 * Runs an iteration of the rc calculations. This function does not call update
 * on the underlying receiver object - user is expected to update the receiver
 * object outside of rc object (and possibly at a different rate). The
 * responsibility of this function is mainly taking care of repeat logic and
 * firing new events for keys that are pressed for a while.
 */
void rc_update(struct rc *self);

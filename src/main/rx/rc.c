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

#include <string.h>

#include "../common/maths.h"
#include "rc.h"
#include "rx.h"

/**
 * @ingroup RX
 * @page RX
 *
 * RC Interface (Radio Control)
 * ----------------------------
 * RC is a higher level interface to the low level RX functions. While RX is
 * able to provide support for reading various receivers, RC set of functions
 * allows you to check if certain stick combinations are active and also
 * supports configurable functions that are triggered when a certain channel is
 * in a certain range. To the application RC module provides an interface
 * similar to that of a keyboard. It allows checking if a certain combination
 * is active using rc_key_pressed() and it also supports callback listener
 * (struct rc_event_listener) through which it can notify the application when
 * a certain key transitions from one state to another.
 */

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


static uint32_t _update_stick_positions(struct rc *self){
	uint32_t stTmp = 0;
	for (int i = 0; i < 4; i++) {
		stTmp >>= 2;
		int16_t chan = rx_get_channel(self->rx, i); 
		if (chan > rxConfig()->mincheck)
			stTmp |= 0x80;  // check for MIN
		if (chan < rxConfig()->maxcheck)
			stTmp |= 0x40;  // check for MAX
	}
	/*
	if (stTmp == rcSticks) {
		if (self->rcDelayCommand < 250)
			self->rcDelayCommand++;
	} else
		self->rcDelayCommand = 0;
		*/
	return stTmp;
}

static bool _range_is_active(struct rc *self, uint8_t auxChannelIndex, channelRange_t *range){
	if (!IS_RANGE_USABLE(range)) {
		return false;
	}

	uint16_t channelValue = constrain(rx_get_channel(self->rx, auxChannelIndex +
		RX_NON_AUX_CHANNEL_COUNT), CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX - 1);
	return (channelValue >= MODE_STEP_TO_CHANNEL_VALUE(range->startStep)
			&& channelValue < MODE_STEP_TO_CHANNEL_VALUE(range->endStep));
}

static uint32_t _update_boxes(struct rc *self, modeActivationCondition_t *modeActivationConditions){
	uint32_t newRcModeMask = 0;

	self->range_mask = 0;
	for (int index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
		modeActivationCondition_t *modeActivationCondition = &modeActivationConditions[index];
		self->range_mask |= 1 << modeActivationCondition->modeId;
		if (_range_is_active(self, modeActivationCondition->auxChannelIndex, &modeActivationCondition->range)) {
			newRcModeMask |= 1 << modeActivationCondition->modeId;
		}
	}
	return newRcModeMask;
}

static bool _box_active(struct rc *self, boxId_e box){
	return !!(self->boxes & box) && (self->range_mask & box);
}

static void _key_down(struct rc *self, rc_key_t key){
	uint8_t byte = key / 8;
	uint8_t bit = key & 0x7;
	self->key_mask[byte] |= (1 << bit);
	self->key_enabled_mask[byte] |= (1 << bit);
}

void rc_update(struct rc *self){
	uint32_t sticks = _update_stick_positions(self);
	self->boxes = _update_boxes(self, modeActivationProfile()->modeActivationConditions);

	rc_command_update(&self->rc_command);

	uint8_t old_mask[sizeof(self->key_mask)];
	memcpy(old_mask, self->key_mask, sizeof(old_mask));
	memset(self->key_mask, 0, sizeof(self->key_mask));

	if(sticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) _key_down(self, RC_KEY_GYROCAL);
	if(sticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) _key_down(self, RC_KEY_ACCCAL);
	if(sticks == THR_HI + YAW_HI + PIT_LO + ROL_CE) _key_down(self, RC_KEY_MAGCAL);
	if(sticks == THR_LO + YAW_LO + PIT_CE + ROL_LO) _key_down(self, RC_KEY_PROFILE1);		  // ROLL left  -> Profile 1
	if(sticks == THR_LO + YAW_LO + PIT_HI + ROL_CE) _key_down(self, RC_KEY_PROFILE2);	 // PITCH up   -> Profile 2
	if(sticks == THR_LO + YAW_LO + PIT_CE + ROL_HI) _key_down(self, RC_KEY_PROFILE3);	 // ROLL right -> Profile 3
	if(sticks == THR_LO + YAW_LO + PIT_LO + ROL_HI) _key_down(self, RC_KEY_SAVE);
	if(sticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) _key_down(self, RC_KEY_STICK_DISARM);
	if(sticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) _key_down(self, RC_KEY_STICK_ARM);
	if(sticks == THR_LO + YAW_CE + PIT_CE + ROL_HI) _key_down(self, RC_KEY_STICK_ARM);
	if(sticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) _key_down(self, RC_KEY_ACC_TRIM_PITCH_INC);
	if(sticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) _key_down(self, RC_KEY_ACC_TRIM_PITCH_DEC);
	if(sticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) _key_down(self, RC_KEY_ACC_TRIM_ROLL_INC);
	if(sticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) _key_down(self, RC_KEY_ACC_TRIM_ROLL_DEC);
	if(sticks == THR_LO + YAW_CE + PIT_HI + ROL_LO) _key_down(self, RC_KEY_DISPLAY_OFF);
	if(sticks == THR_LO + YAW_CE + PIT_HI + ROL_HI) _key_down(self, RC_KEY_DISPLAY_ON);
	if(_box_active(self, BOXARM)) _key_down(self, RC_KEY_FUNC_ARM);
	if(_box_active(self, BOXANGLE)) _key_down(self, RC_KEY_FUNC_LEVEL);
	if(_box_active(self, BOXHORIZON)) _key_down(self, RC_KEY_FUNC_BLEND);

	// compare the old mask to the new mask and fire the events
	for(int c = 0; c < RC_NUM_KEYS; c++){
		uint8_t byte = c / 8;
		uint8_t bit = c & 0x7;
		rc_key_state_t old_state = (old_mask[byte] & (1 << bit))?RC_KEY_PRESSED:RC_KEY_RELEASED;
		rc_key_state_t new_state = (self->key_mask[byte] & (1 << bit))?RC_KEY_PRESSED:RC_KEY_RELEASED;
		if((old_state != new_state) && self->evl && self->evl->on_key_state)
			self->evl->on_key_state(self->evl, c, new_state);
	}
}

void rc_init(struct rc *self, struct rx *rx, struct rc_event_listener *evl){
	memset(self, 0, sizeof(*self));
	self->rx = rx;
	self->evl = evl;
	rc_command_init(&self->rc_command, rx);
	rc_command_set_rate_config(&self->rc_command, controlRateProfiles(0));
}

int16_t rc_get_command(struct rc *self, uint8_t axis){
	// TODO: move rc_command code here..
	return rc_command_axis(&self->rc_command, axis);
}

rc_key_state_t rc_key_state(struct rc *self, rc_key_t key){
	uint8_t byte = key / 8;
	uint8_t bit = key & 0x7;
	bool pressed = !!(self->key_mask[byte] & (1 << bit));
	bool enabled = !!(self->key_enabled_mask[byte] & (1 << bit));
	if(!enabled) return RC_KEY_DISABLED;
	if(pressed) return RC_KEY_PRESSED;
	return RC_KEY_RELEASED;
}

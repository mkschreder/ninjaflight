#include <string.h>

#include "../common/maths.h"
#include "rc.h"
#include "rx.h"

uint32_t _update_stick_positions(struct rc *self){
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

	for (int index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
		modeActivationCondition_t *modeActivationCondition = &modeActivationConditions[index];

		if (_range_is_active(self, modeActivationCondition->auxChannelIndex, &modeActivationCondition->range)) {
			newRcModeMask |= 1 << modeActivationCondition->modeId;
		}
	}
	return newRcModeMask;
}

static bool _box_active(struct rc *self, boxId_e box){
	return !!(self->boxes & box);
}

static void _key_down(struct rc *self, rc_key_t key){
	self->keys |= key;
}

void rc_update(struct rc *self){
	uint32_t sticks = _update_stick_positions(self);
	self->boxes = _update_boxes(self, modeActivationProfile()->modeActivationConditions);

	rc_command_update(&self->rc_command);

	self->keys = 0;

	if(sticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) _key_down(self, RC_KEY_GYROCAL);
	if(sticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) _key_down(self, RC_KEY_ACCCAL);
	if(sticks == THR_HI + YAW_HI + PIT_LO + ROL_CE) _key_down(self, RC_KEY_MAGCAL);
	if(sticks == THR_LO + YAW_LO + PIT_CE + ROL_LO) _key_down(self, RC_KEY_PROFILE1);		  // ROLL left  -> Profile 1
	if(sticks == THR_LO + YAW_LO + PIT_HI + ROL_CE) _key_down(self, RC_KEY_PROFILE2);	 // PITCH up   -> Profile 2
	if(sticks == THR_LO + YAW_LO + PIT_CE + ROL_HI) _key_down(self, RC_KEY_PROFILE3);	 // ROLL right -> Profile 3
	if(sticks == THR_LO + YAW_LO + PIT_LO + ROL_HI) _key_down(self, RC_KEY_SAVE);
	if(sticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) _key_down(self, RC_KEY_DISARM);
	if(sticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) _key_down(self, RC_KEY_ARM);
	if(sticks == THR_LO + YAW_CE + PIT_CE + ROL_HI) _key_down(self, RC_KEY_ARM);
	if(sticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) _key_down(self, RC_KEY_ACC_TRIM_PITCH_INC);
	if(sticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) _key_down(self, RC_KEY_ACC_TRIM_PITCH_DEC);
	if(sticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) _key_down(self, RC_KEY_ACC_TRIM_ROLL_INC);
	if(sticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) _key_down(self, RC_KEY_ACC_TRIM_ROLL_DEC);
	if(sticks == THR_LO + YAW_CE + PIT_HI + ROL_LO) _key_down(self, RC_KEY_DISPLAY_OFF);
	if(sticks == THR_LO + YAW_CE + PIT_HI + ROL_HI) _key_down(self, RC_KEY_DISPLAY_ON);
	if(_box_active(self, BOXARM)) _key_down(self, RC_KEY_ARM);
	if(_box_active(self, BOXANGLE)) _key_down(self, RC_KEY_LEVEL);
	if(_box_active(self, BOXHORIZON)) _key_down(self, RC_KEY_BLEND);
}

void rc_init(struct rc *self, struct rx *rx){
	memset(self, 0, sizeof(*self));
	self->rx = rx;
	rc_command_init(&self->rc_command, rx);
	rc_command_set_rate_config(&self->rc_command, controlRateProfiles(0));
}

int16_t rc_get_command(struct rc *self, uint8_t axis){
	// TODO: move rc_command code here..
	return rc_command_axis(&self->rc_command, axis);
}

bool rc_key_active(struct rc *self, rc_key_t key){
	return !!(self->keys & key);
}

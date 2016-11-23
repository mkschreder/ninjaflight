#include "commander.h"

enum {
	COM_FLAG_PROCESS_RC = (1 << 0)
};

static void _commander_filter_rc(struct commander *self, int32_t dt_us){
	// TODO: fix this
	uint16_t refresh_rate = 20000;//rc_get_refresh_rate();

	int16_t interp_factor = refresh_rate / dt_us;

	if (self->flags & COM_FLAG_PROCESS_RC) {
		for (int channel=0; channel < 4; channel++) {
			self->deltaRC[channel] = self->rcCommand[channel] -  (self->lastCommand[channel] - self->deltaRC[channel] * self->factor / interp_factor);
			self->lastCommand[channel] = self->rcCommand[channel];
		}

		self->flags &= ~COM_FLAG_PROCESS_RC;;
		self->factor = interp_factor - 1;
	}

	// interpolation needs to be done even for first time
	if (self->factor > 0) {
		for (int channel=0; channel < 4; channel++) {
			self->rcCommand[channel] = self->lastCommand[channel] - self->deltaRC[channel] * self->factor/ interp_factor;
		}
		self->factor--;
	}
}

static void ninja_update_controls(struct ninja *self){
	int32_t tmp, tmp2;
	int32_t axis, prop1 = 0, tpa_percent;

	// PITCH & ROLL only dynamic PID adjustment,  depending on throttle value
	if (rc_get_channel_value(THROTTLE) < currentControlRateProfile->tpa_breakpoint) {
		tpa_percent = 100;
	} else {
		if (rc_get_channel_value(THROTTLE) < 2000) {
			tpa_percent = 100 - (uint16_t)currentControlRateProfile->dynThrPID * (rc_get_channel_value(THROTTLE) - currentControlRateProfile->tpa_breakpoint) / (2000 - currentControlRateProfile->tpa_breakpoint);
		} else {
			tpa_percent = 100 - currentControlRateProfile->dynThrPID;
		}
	}

	for (axis = 0; axis < 3; axis++) {
		tmp = MIN(ABS(rc_get_channel_value(axis) - rxConfig()->midrc), 500);
		if (axis == ROLL || axis == PITCH) {
			if (rcControlsConfig()->deadband) {
				if (tmp > rcControlsConfig()->deadband) {
					tmp -= rcControlsConfig()->deadband;
				} else {
					tmp = 0;
				}
			}

			tmp2 = tmp / 100;
			rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
			prop1 = 100 - (uint16_t)currentControlRateProfile->rates[axis] * tmp / 500;
			prop1 = (uint16_t)prop1 * tpa_percent / 100;
		} else if (axis == YAW) {
			if (rcControlsConfig()->yaw_deadband) {
				if (tmp > rcControlsConfig()->yaw_deadband) {
					tmp -= rcControlsConfig()->yaw_deadband;
				} else {
					tmp = 0;
				}
			}
			tmp2 = tmp / 100;
			rcCommand[axis] = (lookupYawRC[tmp2] + (tmp - tmp2 * 100) * (lookupYawRC[tmp2 + 1] - lookupYawRC[tmp2]) / 100) * -rcControlsConfig()->yaw_control_direction;
			prop1 = 100 - (uint16_t)currentControlRateProfile->rates[axis] * ABS(tmp) / 500;
		}
#ifndef SKIP_PID_MW23
		// FIXME axis indexes into pids.  use something like lookupPidIndex(rc_alias_e alias) to reduce coupling.
		// TODO: move this kind of shit into pid controller module
		anglerate_set_pid_axis_scale(&default_controller, axis, prop1);
#endif

		// non coupled PID reduction scaler used in PID controller 1 and PID controller 2. YAW TPA disabled. 100 means 100% of the pids
		if (axis == YAW) {
			anglerate_set_pid_axis_weight(&default_controller, axis, 100);
		}
		else {
			anglerate_set_pid_axis_weight(&default_controller, axis, tpa_percent);
		}

		if (rc_get_channel_value(axis) < rxConfig()->midrc)
			rcCommand[axis] = -rcCommand[axis];
	}

	tmp = constrain(rc_get_channel_value(THROTTLE), rxConfig()->mincheck, PWM_RANGE_MAX);
	tmp = (uint32_t)(tmp - rxConfig()->mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - rxConfig()->mincheck);	   // [MINCHECK;2000] -> [0;1000]
	tmp2 = tmp / 100;
	rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;	// [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

	if (rxConfig()->rcSmoothing) {
		filterRc(dt);
	}

    // If we're armed, at minimum throttle, and we do arming via the
    // sticks, do not process yaw input from the rx.  We do this so the
    // motors do not spin up while we are trying to arm or disarm.
    // Allow yaw control for tricopters if the user wants the servo to move even when unarmed.
    if (isUsingSticksForArming() && rc_get_channel_value(THROTTLE) <= rxConfig()->mincheck
#ifdef USE_SERVOS
                && !((mixerConfig()->mixerMode == MIXER_TRI || mixerConfig()->mixerMode == MIXER_CUSTOM_TRI) && mixerConfig()->tri_unarmed_servo)
#endif
                && mixerConfig()->mixerMode != MIXER_AIRPLANE
                && mixerConfig()->mixerMode != MIXER_FLYING_WING
    ) {
        rcCommand[YAW] = 0;
    }
}

#define AIRMODE_DEADBAND 12

// true if arming is done via the sticks (as opposed to a switch)
static bool isUsingSticksToArm = true;

int16_t rcCommand[4];		   // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW

uint32_t rcModeActivationMask; // one bit per mode defined in boxId_e


bool isUsingSticksForArming(void)
{
	return isUsingSticksToArm;
}


bool areSticksInApModePosition(uint16_t ap_mode)
{
	return ABS(rcCommand[ROLL]) < ap_mode && ABS(rcCommand[PITCH]) < ap_mode;
}

throttleStatus_e calculateThrottleStatus(rxConfig_t *rxConfig, uint16_t deadband3d_throttle)
{
	int16_t throttle = rc_get_channel_value(THROTTLE); 
	if (feature(FEATURE_3D) && (throttle > (rxConfig->midrc - deadband3d_throttle) && throttle < (rxConfig->midrc + deadband3d_throttle)))
		return THROTTLE_LOW;
	else if (!feature(FEATURE_3D) && (throttle < rxConfig->mincheck))
		return THROTTLE_LOW;

	return THROTTLE_HIGH;
}

rollPitchStatus_e calculateRollPitchCenterStatus(rxConfig_t *rxConfig)
{
	int16_t pitch = rc_get_channel_value(PITCH); 
	int16_t roll = rc_get_channel_value(ROLL); 
	if (((pitch < (rxConfig->midrc + AIRMODE_DEADBAND)) && (pitch > (rxConfig->midrc -AIRMODE_DEADBAND)))
			&& ((roll < (rxConfig->midrc + AIRMODE_DEADBAND)) && (roll > (rxConfig->midrc -AIRMODE_DEADBAND))))
		return CENTERED;

	return NOT_CENTERED;
}

void processRcStickPositions(rxConfig_t *rxConfig, throttleStatus_e throttleStatus, bool retarded_arm, bool disarm_kill_switch)
{
	static uint8_t rcDelayCommand;	  // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
	static uint8_t rcSticks;			// this hold sticks position for command combos
	uint8_t stTmp = 0;
	int i;

	// ------------------ STICKS COMMAND HANDLER --------------------
	// checking sticks positions
	for (i = 0; i < 4; i++) {
		stTmp >>= 2;
		int16_t chan = rc_get_channel_value(i); 
		if (chan > rxConfig->mincheck)
			stTmp |= 0x80;  // check for MIN
		if (chan < rxConfig->maxcheck)
			stTmp |= 0x40;  // check for MAX
	}
	if (stTmp == rcSticks) {
		if (rcDelayCommand < 250)
			rcDelayCommand++;
	} else
		rcDelayCommand = 0;
	rcSticks = stTmp;

	// perform actions
	if (!isUsingSticksToArm) {

		if (rcModeIsActive(BOXARM)) {
			// Arming via ARM BOX
			if (throttleStatus == THROTTLE_LOW) {
				if (ARMING_FLAG(OK_TO_ARM)) {
					mwArm();
				}
			}
		} else {
			// Disarming via ARM BOX

			if (ARMING_FLAG(ARMED) && rxIsReceivingSignal() && !failsafeIsActive()  ) {
				if (disarm_kill_switch) {
					mwDisarm();
				} else if (throttleStatus == THROTTLE_LOW) {
					mwDisarm();
				}
			}
		}
	}

	if (rcDelayCommand != 20) {
		return;
	}

	if (isUsingSticksToArm) {
		// Disarm on throttle down + yaw
		if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) {
			if (ARMING_FLAG(ARMED))
				mwDisarm();
			else {
				beeper(BEEPER_DISARM_REPEAT);	// sound tone while stick held
				rcDelayCommand = 0;			  // reset so disarm tone will repeat
			}
		}
			// Disarm on roll (only when retarded_arm is enabled)
		if (retarded_arm && (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_LO)) {
			if (ARMING_FLAG(ARMED))
				mwDisarm();
			else {
				beeper(BEEPER_DISARM_REPEAT);	// sound tone while stick held
				rcDelayCommand = 0;			  // reset so disarm tone will repeat
			}
		}
	}

	if (ARMING_FLAG(ARMED)) {
		// actions during armed
		return;
	}

	// actions during not armed
	i = 0;

	if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
		// GYRO calibration
		// TODO: reenable calibration
		//gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);

#ifdef GPS
		if (feature(FEATURE_GPS)) {
			GPS_reset_home_position();
		}
#endif

#ifdef BARO
		if (sensors(SENSOR_BARO))
			baroSetCalibrationCycles(10); // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)
#endif

		return;
	}

	if (feature(FEATURE_INFLIGHT_ACC_CAL) && (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_HI)) {
		// Inflight ACC Calibration
		handleInflightCalibrationStickPosition();
		return;
	}

	// Multiple configuration profiles
	if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_LO)		  // ROLL left  -> Profile 1
		i = 1;
	else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_CE)	 // PITCH up   -> Profile 2
		i = 2;
	else if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI)	 // ROLL right -> Profile 3
		i = 3;
	if (i) {
		changeProfile(i - 1);
		beeperConfirmationBeeps(i);
		return;
	}

	if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_HI) {
		saveConfigAndNotify();
	}

	if (isUsingSticksToArm) {

		if (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) {
			// Arm via YAW
			mwArm();
			return;
		}

		if (retarded_arm && (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_HI)) {
			// Arm via ROLL
			mwArm();
			return;
		}
	}

	if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) {
		// Calibrating Acc
		ins_start_acc_calibration(&default_ins);
		return;
	}


	if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE) {
		// Calibrating Mag
		ENABLE_STATE(CALIBRATE_MAG);
		return;
	}


	// Accelerometer Trim

	rollAndPitchTrims_t accelerometerTrimsDelta;
	memset(&accelerometerTrimsDelta, 0, sizeof(accelerometerTrimsDelta));

	bool shouldApplyRollAndPitchTrimDelta = false;
	if (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {
		accelerometerTrimsDelta.values.pitch = 2;
		shouldApplyRollAndPitchTrimDelta = true;
	} else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {
		accelerometerTrimsDelta.values.pitch = -2;
		shouldApplyRollAndPitchTrimDelta = true;
	} else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {
		accelerometerTrimsDelta.values.roll = 2;
		shouldApplyRollAndPitchTrimDelta = true;
	} else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {
		accelerometerTrimsDelta.values.roll = -2;
		shouldApplyRollAndPitchTrimDelta = true;
	}
	if (shouldApplyRollAndPitchTrimDelta) {
		applyAndSaveAccelerometerTrimsDelta(&accelerometerTrimsDelta);
		rcDelayCommand = 0; // allow autorepetition
		return;
	}

#ifdef DISPLAY
	if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_LO) {
		displayDisablePageCycling();
	}

	if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_HI) {
		displayEnablePageCycling();
	}
#endif

}

bool rcModeIsActivationConditionPresent(modeActivationCondition_t *modeActivationConditions, boxId_e modeId)
{
	uint8_t index;

	for (index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
		modeActivationCondition_t *modeActivationCondition = &modeActivationConditions[index];

		if (modeActivationCondition->modeId == modeId && IS_RANGE_USABLE(&modeActivationCondition->range)) {
			return true;
		}
	}
	return false;
}

bool isRangeActive(uint8_t auxChannelIndex, channelRange_t *range)
{
	if (!IS_RANGE_USABLE(range)) {
		return false;
	}

	uint16_t channelValue = constrain(rc_get_channel_value(auxChannelIndex + NON_AUX_CHANNEL_COUNT), CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX - 1);
	return (channelValue >= MODE_STEP_TO_CHANNEL_VALUE(range->startStep)
			&& channelValue < MODE_STEP_TO_CHANNEL_VALUE(range->endStep));
}

bool rcModeIsActive(boxId_e modeId)
{
	return rcModeActivationMask & (1 << modeId);
}

void rcModeUpdateActivated(modeActivationCondition_t *modeActivationConditions)
{
	uint32_t newRcModeMask = 0;

	for (int index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
		modeActivationCondition_t *modeActivationCondition = &modeActivationConditions[index];

		if (isRangeActive(modeActivationCondition->auxChannelIndex, &modeActivationCondition->range)) {
			newRcModeMask |= 1 << modeActivationCondition->modeId;
		}
	}
	rcModeActivationMask = newRcModeMask;
}

int32_t getRcStickDeflection(int32_t axis, uint16_t midrc)
{
	return MIN(ABS(rc_get_channel_value(axis) - midrc), 500);
}

void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions)
{
	isUsingSticksToArm = !rcModeIsActivationConditionPresent(modeActivationConditions, BOXARM);
}


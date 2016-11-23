#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"
#include "common/filter.h"

#include "config/parameter_group.h"
#include "config/tilt.h"

#include "io/rc_adjustments.h"

#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/initialisation.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/rc_curves.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/transponder_ir.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/rate_profile.h"
#include "flight/mixer.h"
#include "flight/anglerate.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/gtune.h"
#include "flight/navigation.h"
#include "flight/tilt.h"
#include "sensors/instruments.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/feature.h"

#include "ninjaflight.h"
#include "ninja.h"
#include "ninja_sched.h"

uint32_t millis(void);

#define AIRMODE_DEADBAND 12
// true if arming is done via the sticks (as opposed to a switch)
static bool isUsingSticksToArm = true;


uint32_t rcModeActivationMask; // one bit per mode defined in boxId_e

/*
This function processes RX dependent coefficients when new RX commands are available
Those are: TPA, throttle expo
*/
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

void applyAndSaveAccelerometerTrimsDelta(rollAndPitchTrims_t *rollAndPitchTrimsDelta)
{
    accelerometerConfig()->trims.values.roll += rollAndPitchTrimsDelta->values.roll;
    accelerometerConfig()->trims.values.pitch += rollAndPitchTrimsDelta->values.pitch;

    //saveConfigAndNotify();
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

// Automatic ACC Offset Calibration
bool AccInflightCalibrationArmed = false;
bool AccInflightCalibrationMeasurementDone = false;
bool AccInflightCalibrationSavetoEEProm = false;
bool AccInflightCalibrationActive = false;
uint16_t InflightcalibratingA = 0;

void handleInflightCalibrationStickPosition(void)
{
    if (AccInflightCalibrationMeasurementDone) {
        // trigger saving into eeprom after landing
        AccInflightCalibrationMeasurementDone = false;
        AccInflightCalibrationSavetoEEProm = true;
    } else {
        AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
        if (AccInflightCalibrationArmed) {
            beeper(BEEPER_ACC_CALIBRATION);
        } else {
            beeper(BEEPER_ACC_CALIBRATION_FAIL);
        }
    }
}

static void updateInflightCalibrationState(void)
{
    if (AccInflightCalibrationArmed && ARMING_FLAG(ARMED) && rc_get_channel_value(THROTTLE) > rxConfig()->mincheck && !rcModeIsActive(BOXARM)) {   // Copter is airborne and you are turning it off via boxarm : start measurement
        InflightcalibratingA = 50;
        AccInflightCalibrationArmed = false;
    }
    if (rcModeIsActive(BOXCALIB)) {      // Use the Calib Option to activate : Calib = TRUE measurement started, Land and Calib = 0 measurement stored
        if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone)
            InflightcalibratingA = 50;
        AccInflightCalibrationActive = true;
    } else if (AccInflightCalibrationMeasurementDone && !ARMING_FLAG(ARMED)) {
        AccInflightCalibrationMeasurementDone = false;
        AccInflightCalibrationSavetoEEProm = true;
    }
}

void ninja_process_rc_sticks(struct ninja *self, rxConfig_t *rxConfig, throttleStatus_e throttleStatus, bool retarded_arm, bool disarm_kill_switch){
    static uint8_t rcDelayCommand;      // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
    static uint8_t rcSticks;            // this hold sticks position for command combos
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
                    ninja_arm(self);
                }
            }
        } else {
            // Disarming via ARM BOX

            if (ARMING_FLAG(ARMED) && rxIsReceivingSignal() && !failsafeIsActive()  ) {
                if (disarm_kill_switch) {
                    ninja_disarm(self);
                } else if (throttleStatus == THROTTLE_LOW) {
                    ninja_disarm(self);
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
                ninja_disarm(self);
            else {
                beeper(BEEPER_DISARM_REPEAT);    // sound tone while stick held
                rcDelayCommand = 0;              // reset so disarm tone will repeat
            }
        }
            // Disarm on roll (only when retarded_arm is enabled)
        if (retarded_arm && (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_LO)) {
            if (ARMING_FLAG(ARMED))
                ninja_disarm(self);
            else {
                beeper(BEEPER_DISARM_REPEAT);    // sound tone while stick held
                rcDelayCommand = 0;              // reset so disarm tone will repeat
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
    if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_LO)          // ROLL left  -> Profile 1
        i = 1;
    else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_CE)     // PITCH up   -> Profile 2
        i = 2;
    else if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI)     // ROLL right -> Profile 3
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
            ninja_arm(self);
            return;
        }

        if (retarded_arm && (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_HI)) {
            // Arm via ROLL
            ninja_arm(self);
            return;
        }
    }

    if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) {
        // Calibrating Acc
		ins_start_acc_calibration(&self->ins);
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

#include <stdio.h>
void ninja_process_rx(struct ninja *self){
    static bool armedBeeperOn = false;
	
	int32_t currentTime = self->syscalls->time.micros(&self->syscalls->time);

	rc_command_update(&self->rc_command);

	rcCommand[ROLL] = rc_command_axis(&self->rc_command, ROLL);
	rcCommand[PITCH] = rc_command_axis(&self->rc_command, PITCH);
	rcCommand[YAW] = rc_command_axis(&self->rc_command, YAW);
	rcCommand[THROTTLE] = rc_command_axis(&self->rc_command, THROTTLE);

	printf("rx: %d %d %d %d\n", rcCommand[ROLL], rcCommand[PITCH], rcCommand[YAW], rcCommand[THROTTLE]);

	// TODO: this should be called read_rx or something
    //calculateRxChannelsAndUpdateFailsafe(currentTime);

	// TODO: this is just here to remember that it needs to be refactored
	self->isRXDataNew = true;


    // in 3D mode, we need to be able to disarm by switch at any time
    if (feature(FEATURE_3D)) {
        if (!rcModeIsActive(BOXARM))
            ninja_disarm(self);
    }

    updateRSSI(currentTime);

    if (feature(FEATURE_FAILSAFE)) {

        if (currentTime > FAILSAFE_POWER_ON_DELAY_US && !failsafeIsMonitoring()) {
            failsafeStartMonitoring();
        }

        failsafeUpdateState();
    }

    throttleStatus_e throttleStatus = calculateThrottleStatus(rxConfig(), rcControlsConfig()->deadband3d_throttle);
    rollPitchStatus_e rollPitchStatus =  calculateRollPitchCenterStatus(rxConfig());

    /* In airmode Iterm should be prevented to grow when Low thottle and Roll + Pitch Centered.
     This is needed to prevent Iterm winding on the ground, but keep full stabilisation on 0 throttle while in air
     Low Throttle + roll and Pitch centered is assuming the copter is on the ground. Done to prevent complex air/ground detections */
    if (throttleStatus == THROTTLE_LOW) {
        if (rcModeIsActive(BOXAIRMODE) && !failsafeIsActive() && ARMING_FLAG(ARMED)) {
            if (rollPitchStatus == CENTERED) {
				anglerate_enable_antiwindup(&self->ctrl, true);
            } else {
				anglerate_enable_antiwindup(&self->ctrl, false);
            }
        } else {
#ifndef SKIP_PID_MW23
            anglerate_reset_angle_i(&self->ctrl);
#endif
            anglerate_reset_rate_i(&self->ctrl);
        }
    } else if(mixer_motor_limit_reached(&self->mixer)){
		// when motor limit reached then we always enable antiwindup (this is taken from cleanflight pid code)
		anglerate_enable_antiwindup(&self->ctrl, true);
	} else {
		anglerate_enable_antiwindup(&self->ctrl, false);
    }

	// plimit is enabled in cleanflight for some reason based on number of motors
	if(mixer_get_motor_count(&self->mixer) >= 4)
		anglerate_enable_plimit(&self->ctrl, true);
	else
		anglerate_enable_plimit(&self->ctrl, false);

    // When armed and motors aren't spinning, do beeps and then disarm
    // board after delay so users without buzzer won't lose fingers.
    // mixTable constrains motor commands, so checking  throttleStatus is enough
    if (ARMING_FLAG(ARMED)
        && feature(FEATURE_MOTOR_STOP)
        && !STATE(FIXED_WING)
    ) {
        if (isUsingSticksForArming()) {
            if (throttleStatus == THROTTLE_LOW) {
                if (armingConfig()->auto_disarm_delay != 0
                    && (int32_t)(self->disarmAt - millis()) < 0
                ) {
                    // auto-disarm configured and delay is over
                    ninja_disarm(self);
                    armedBeeperOn = false;
                } else {
                    // still armed; do warning beeps while armed
                    beeper(BEEPER_ARMED);
                    armedBeeperOn = true;
                }
            } else {
                // throttle is not low
                if (armingConfig()->auto_disarm_delay != 0) {
                    // extend disarm time
                    self->disarmAt = millis() + armingConfig()->auto_disarm_delay * 1000;
                }

                if (armedBeeperOn) {
                    beeperSilence();
                    armedBeeperOn = false;
                }
            }
        } else {
            // arming is via AUX switch; beep while throttle low
            if (throttleStatus == THROTTLE_LOW) {
                beeper(BEEPER_ARMED);
                armedBeeperOn = true;
            } else if (armedBeeperOn) {
                beeperSilence();
                armedBeeperOn = false;
            }
        }
    }

    ninja_process_rc_sticks(self, rxConfig(), throttleStatus, armingConfig()->retarded_arm, armingConfig()->disarm_kill_switch);

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        updateInflightCalibrationState();
    }

    rcModeUpdateActivated(modeActivationProfile()->modeActivationConditions);

    if (!cliMode) {
        updateAdjustmentStates(adjustmentProfile()->adjustmentRanges);
        //processRcAdjustments(currentControlRateProfile, rxConfig());
    }

    bool canUseHorizonMode = true;

    if ((rcModeIsActive(BOXANGLE) || (feature(FEATURE_FAILSAFE) && failsafeIsActive())) && (sensors(SENSOR_ACC))) {
        // bumpless transfer to Level mode
        canUseHorizonMode = false;

        if (!FLIGHT_MODE(ANGLE_MODE)) {
#ifndef SKIP_PID_MW23
            anglerate_reset_angle_i(&self->ctrl);
#endif
            ENABLE_FLIGHT_MODE(ANGLE_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(ANGLE_MODE); // failsafe support
    }

    if (rcModeIsActive(BOXHORIZON) && canUseHorizonMode) {

        DISABLE_FLIGHT_MODE(ANGLE_MODE);

        if (!FLIGHT_MODE(HORIZON_MODE)) {
#ifndef SKIP_PID_MW23
            anglerate_reset_angle_i(&self->ctrl);
#endif
            ENABLE_FLIGHT_MODE(HORIZON_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(HORIZON_MODE);
    }

    #ifdef  MAG
	// TODO: refactor the mag hold mode
    if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
        if (rcModeIsActive(BOXMAG)) {
            if (!FLIGHT_MODE(MAG_MODE)) {
                ENABLE_FLIGHT_MODE(MAG_MODE);
                self->magHold = DECIDEGREES_TO_DEGREES(ins_get_yaw_dd(&self->ins));
            }
        } else {
            DISABLE_FLIGHT_MODE(MAG_MODE);
        }
        if (rcModeIsActive(BOXHEADFREE)) {
            if (!FLIGHT_MODE(HEADFREE_MODE)) {
                ENABLE_FLIGHT_MODE(HEADFREE_MODE);
            }
        } else {
            DISABLE_FLIGHT_MODE(HEADFREE_MODE);
        }
        if (rcModeIsActive(BOXHEADADJ)) {
            headFreeModeHold = DECIDEGREES_TO_DEGREES(ins_get_yaw_dd(&self->ins)); // acquire new heading
        }
    }
#endif

#ifdef GPS
    if (sensors(SENSOR_GPS)) {
        updateGpsWaypointsAndMode();
    }
#endif

    if (rcModeIsActive(BOXPASSTHRU)) {
        ENABLE_FLIGHT_MODE(PASSTHRU_MODE);
    } else {
        DISABLE_FLIGHT_MODE(PASSTHRU_MODE);
    }

    if (mixerConfig()->mixerMode == MIXER_FLYING_WING || mixerConfig()->mixerMode == MIXER_AIRPLANE) {
        DISABLE_FLIGHT_MODE(HEADFREE_MODE);
    }

#ifdef TELEMETRY
    if (feature(FEATURE_TELEMETRY)) {
        if ((!telemetryConfig()->telemetry_switch && ARMING_FLAG(ARMED))
            || (telemetryConfig()->telemetry_switch && rcModeIsActive(BOXTELEMETRY))) {
			// TODO: uncommend this later
            //releaseSharedTelemetryPorts();
        } else {
            // the telemetry state must be checked immediately so that shared serial ports are released.
            telemetryCheckState();
            mspSerialAllocatePorts();
        }
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



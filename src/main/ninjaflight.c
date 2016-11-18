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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include <platform.h>

#include "scheduler.h"
#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"
#include "common/filter.h"

#include "config/parameter_group.h"
#include "config/tilt.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/light_led.h"

//#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/gyro_sync.h"
#include "drivers/pwm_output.h"
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

// June 2013     V2.2-dev

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

/* VBAT monitoring interval (in microseconds) - 1s*/
#define VBATINTERVAL (6 * 3500)
/* IBat monitoring interval (in microseconds) - 6 default looptimes */
#define IBATINTERVAL (6 * 3500)

// TODO: remove this once we have refactored enough to pass self to this module
//static struct ninja _default_fc;
//static struct ninja *self = &_default_fc;

float dT;

int16_t magHold;
int16_t headFreeModeHold;

uint8_t motorControlEnable = false;

int16_t telemTemperature1;      // gyro sensor temperature
static uint32_t disarmAt;     // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero

extern uint32_t currentTime;
extern uint8_t PIDweight[3];

static bool isRXDataNew;

void ninja_init(struct ninja *self){
	self->placeholder = 0;
}

void applyAndSaveAccelerometerTrimsDelta(rollAndPitchTrims_t *rollAndPitchTrimsDelta)
{
    accelerometerConfig()->trims.values.roll += rollAndPitchTrimsDelta->values.roll;
    accelerometerConfig()->trims.values.pitch += rollAndPitchTrimsDelta->values.pitch;

    saveConfigAndNotify();
}

#ifdef GTUNE

static void updateGtuneState(void)
{
    static bool GTuneWasUsed = false;

    if (rcModeIsActive(BOXGTUNE)) {
        if (!FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            ENABLE_FLIGHT_MODE(GTUNE_MODE);
            init_Gtune();
            GTuneWasUsed = true;
        }
        if (!FLIGHT_MODE(GTUNE_MODE) && !ARMING_FLAG(ARMED) && GTuneWasUsed) {
            saveConfigAndNotify();
            GTuneWasUsed = false;
        }
    } else {
        if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            DISABLE_FLIGHT_MODE(GTUNE_MODE);
        }
    }
}
#endif

bool isCalibrating(void)
{
	return ins_is_calibrated(&default_ins);
}

/*
This function processes RX dependent coefficients when new RX commands are available
Those are: TPA, throttle expo
*/
static void updateRcCommands(void)
{

    int32_t tmp, tmp2;
    int32_t axis, prop1 = 0, prop2;

    // PITCH & ROLL only dynamic PID adjustment,  depending on throttle value
    if (rc_get_channel_value(THROTTLE) < currentControlRateProfile->tpa_breakpoint) {
        prop2 = 100;
    } else {
        if (rc_get_channel_value(THROTTLE) < 2000) {
			uint16_t t = rc_get_channel_value(THROTTLE) - currentControlRateProfile->tpa_breakpoint;
            prop2 = 100 - (uint16_t)currentControlRateProfile->dynThrPID * t / constrain(2000 - currentControlRateProfile->tpa_breakpoint, 1000, 2000);
        } else {
            prop2 = 100 - currentControlRateProfile->dynThrPID;
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
            prop1 = (uint16_t)prop1 * prop2 / 100;
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
			anglerate_set_pid_axis_weight(&default_controller, axis, prop2);
        }

        if (rc_get_channel_value(axis) < rxConfig()->midrc)
            rcCommand[axis] = -rcCommand[axis];
    }

    tmp = constrain(rc_get_channel_value(THROTTLE), rxConfig()->mincheck, PWM_RANGE_MAX);
    tmp = (uint32_t)(tmp - rxConfig()->mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - rxConfig()->mincheck);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

    if (FLIGHT_MODE(HEADFREE_MODE)) {
        float radDiff = degreesToRadians(DECIDEGREES_TO_DEGREES(ins_get_yaw_dd(&default_ins)) - headFreeModeHold);
        float cosDiff = cos_approx(radDiff);
        float sinDiff = sin_approx(radDiff);
        int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
        rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
        rcCommand[PITCH] = rcCommand_PITCH;
    }
}

static uint8_t calibrating = 0;
void ninja_calibrate_acc(void){
	ins_start_acc_calibration(&default_ins);
	calibrating = 1;
}

void ninja_calibrate_mag(void){
	ins_start_mag_calibration(&default_ins);
	calibrating = 1;
}

static void updateLEDs(void)
{
    if (ARMING_FLAG(ARMED)) {
        led_on(0);
    } else {
        if (rcModeIsActive(BOXARM) == 0) {
            ENABLE_ARMING_FLAG(OK_TO_ARM);
        }

/*
		// TODO: for now allow arming when not leveled but rethink this logic entirely
        if (!imu_is_leveled(&default_imu, armingConfig()->max_arm_angle)) {
            DISABLE_ARMING_FLAG(OK_TO_ARM);
        }
*/
		// TODO: this should probably be checked elsewhere
		if(calibrating && !isCalibrating()){
			// save config here since calibration may have changed it
			saveConfigAndNotify();
			calibrating = 0;
		}

        if (isCalibrating() || isSystemOverloaded()) {
            warningLedFlash();
            DISABLE_ARMING_FLAG(OK_TO_ARM);
        } else {
            if (ARMING_FLAG(OK_TO_ARM)) {
                warningLedDisable();
            } else {
                warningLedFlash();
            }
        }

        warningLedUpdate();
    }
}

void mwDisarm(void)
{
    if (ARMING_FLAG(ARMED)) {
		// reset mixer minimum values in case something else has changed them
		mixer_enable_armed(&default_mixer, false);

        DISABLE_ARMING_FLAG(ARMED);

#ifdef BLACKBOX
        if (feature(FEATURE_BLACKBOX)) {
            finishBlackbox();
        }
#endif

        beeper(BEEPER_DISARMING);      // emit disarm tone
    }
}

#define TELEMETRY_FUNCTION_MASK (FUNCTION_TELEMETRY_FRSKY | FUNCTION_TELEMETRY_HOTT | FUNCTION_TELEMETRY_SMARTPORT | FUNCTION_TELEMETRY_LTM | FUNCTION_TELEMETRY_MAVLINK)

#ifdef TELEMETRY
static void releaseSharedTelemetryPorts(void) {
    serialPort_t *sharedPort = findSharedSerialPort(TELEMETRY_FUNCTION_MASK, FUNCTION_MSP);
     while (sharedPort) {
         mspSerialReleasePortIfAllocated(sharedPort);
         sharedPort = findNextSharedSerialPort(TELEMETRY_FUNCTION_MASK, FUNCTION_MSP);
     }
}
#endif

void mwArm(void)
{
    if (ARMING_FLAG(OK_TO_ARM)) {
        if (ARMING_FLAG(ARMED)) {
            return;
        }
        if (rcModeIsActive(BOXFAILSAFE)) {
            return;
        }
        if (!ARMING_FLAG(PREVENT_ARMING)) {
            ENABLE_ARMING_FLAG(ARMED);
            headFreeModeHold = DECIDEGREES_TO_DEGREES(ins_get_yaw_dd(&default_ins));
			mixer_enable_armed(&default_mixer, true);

#ifdef BLACKBOX
            if (feature(FEATURE_BLACKBOX)) {
                serialPort_t *sharedBlackboxAndMspPort = findSharedSerialPort(FUNCTION_BLACKBOX, FUNCTION_MSP);
                if (sharedBlackboxAndMspPort) {
                    mspSerialReleasePortIfAllocated(sharedBlackboxAndMspPort);
                }
                startBlackbox();
            }
#endif
            disarmAt = millis() + armingConfig()->auto_disarm_delay * 1000;   // start disarm timeout, will be extended when throttle is nonzero

            //beep to indicate arming
#ifdef GPS
            if (feature(FEATURE_GPS) && STATE(GPS_FIX) && GPS_numSat >= 5)
                beeper(BEEPER_ARMING_GPS_FIX);
            else
                beeper(BEEPER_ARMING);
#else
            beeper(BEEPER_ARMING);
#endif

            return;
        }
    }

    if (!ARMING_FLAG(ARMED)) {
        beeperConfirmationBeeps(1);
    }
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

static void updateMagHold(void)
{
	// TODO: really refactor this kind of crap. This belongs in flight control code
    if (ABS(rcCommand[YAW]) < 15 && FLIGHT_MODE(MAG_MODE)) {
        int16_t dif = DECIDEGREES_TO_DEGREES(ins_get_yaw_dd(&default_ins)) - magHold;
        if (dif <= -180)
            dif += 360;
        if (dif >= +180)
            dif -= 360;
        dif *= -rcControlsConfig()->yaw_control_direction;
		// TODO: small angle was set in imu before. Generally this kind of logic is horrible. This needs to be scrapped.
        if (STATE(SMALL_ANGLE))
            rcCommand[YAW] -= dif * pidProfile()->P8[PIDMAG] / 30;    // 18 deg
    } else
        magHold = DECIDEGREES_TO_DEGREES(ins_get_yaw_dd(&default_ins));
}

static void processRx(void)
{
    static bool armedBeeperOn = false;

    calculateRxChannelsAndUpdateFailsafe(currentTime);

    // in 3D mode, we need to be able to disarm by switch at any time
    if (feature(FEATURE_3D)) {
        if (!rcModeIsActive(BOXARM))
            mwDisarm();
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
				anglerate_enable_antiwindup(&default_controller, true);
            } else {
				anglerate_enable_antiwindup(&default_controller, false);
            }
        } else {
#ifndef SKIP_PID_MW23
            anglerate_reset_angle_i(&default_controller);
#endif
            anglerate_reset_rate_i(&default_controller);
        }
    } else if(mixer_motor_limit_reached(&default_mixer)){
		// when motor limit reached then we always enable antiwindup (this is taken from cleanflight pid code)
		anglerate_enable_antiwindup(&default_controller, true);
	} else {
		anglerate_enable_antiwindup(&default_controller, false);
    }

	// plimit is enabled in cleanflight for some reason based on number of motors
	if(mixer_get_motor_count(&default_mixer) >= 4)
		anglerate_enable_plimit(&default_controller, true);
	else
		anglerate_enable_plimit(&default_controller, false);

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
                    && (int32_t)(disarmAt - millis()) < 0
                ) {
                    // auto-disarm configured and delay is over
                    mwDisarm();
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
                    disarmAt = millis() + armingConfig()->auto_disarm_delay * 1000;
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

    processRcStickPositions(rxConfig(), throttleStatus, armingConfig()->retarded_arm, armingConfig()->disarm_kill_switch);

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        updateInflightCalibrationState();
    }

    rcModeUpdateActivated(modeActivationProfile()->modeActivationConditions);

    if (!cliMode) {
        updateAdjustmentStates(adjustmentProfile()->adjustmentRanges);
        processRcAdjustments(currentControlRateProfile, rxConfig());
    }

    bool canUseHorizonMode = true;

    if ((rcModeIsActive(BOXANGLE) || (feature(FEATURE_FAILSAFE) && failsafeIsActive())) && (sensors(SENSOR_ACC))) {
        // bumpless transfer to Level mode
        canUseHorizonMode = false;

        if (!FLIGHT_MODE(ANGLE_MODE)) {
#ifndef SKIP_PID_MW23
            anglerate_reset_angle_i(&default_controller);
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
            anglerate_reset_angle_i(&default_controller);
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
                magHold = DECIDEGREES_TO_DEGREES(ins_get_yaw_dd(&default_ins));
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
            headFreeModeHold = DECIDEGREES_TO_DEGREES(ins_get_yaw_dd(&default_ins)); // acquire new heading
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
            releaseSharedTelemetryPorts();
        } else {
            // the telemetry state must be checked immediately so that shared serial ports are released.
            telemetryCheckState();
            mspSerialAllocatePorts();
        }
    }
#endif

}

static void filterRc(float dt){
    static int16_t lastCommand[4] = { 0, 0, 0, 0 };
    static int16_t deltaRC[4] = { 0, 0, 0, 0 };
    static int16_t factor, rcInterpolationFactor;
    uint16_t rxRefreshRate = rc_get_refresh_rate();

    rcInterpolationFactor = rxRefreshRate / (dt * 1000000UL);

    if (isRXDataNew) {
        for (int channel=0; channel < 4; channel++) {
            deltaRC[channel] = rcCommand[channel] -  (lastCommand[channel] - deltaRC[channel] * factor / rcInterpolationFactor);
            lastCommand[channel] = rcCommand[channel];
        }

        isRXDataNew = false;
        factor = rcInterpolationFactor - 1;
    } else {
        factor--;
    }

    // Interpolate steps of rcCommand
    if (factor > 0) {
        for (int channel=0; channel < 4; channel++) {
            rcCommand[channel] = lastCommand[channel] - deltaRC[channel] * factor/rcInterpolationFactor;
         }
    } else {
        factor = 0;
    }
}

#if defined(BARO) || defined(SONAR)
static bool haveUpdatedRcCommandsOnce = false;
#endif


// TODO: untangle all the crap with dt
void ninja_run_pid_loop(struct ninja *self, float dt_){
	UNUSED(self);
	UNUSED(dt_);

    currentTime = micros();
	static uint32_t previousIMUUpdateTime = 0;
    float dt = (currentTime - previousIMUUpdateTime) * 1e-6f;
	dT = dt;
    previousIMUUpdateTime = currentTime;

	int16_t rawgyro[3];
	if (gyro.read(rawgyro)) {
		ins_process_gyro(&default_ins, rawgyro[0], rawgyro[1], rawgyro[2]);
	}

#if defined(GPS)
/*
	// TODO: rethink how to enter gps yaw angle into ins (probably we enter all gps data and then ins decides how to use the angle!)
    if (STATE(FIXED_WING) && sensors(SENSOR_GPS) && STATE(GPS_FIX) && GPS_numSat >= 5 && GPS_speed >= 300) {
        // In case of a fixed-wing aircraft we can use GPS course over ground to correct heading
        ins_input_yaw_dd(&default_imu, GPS_ground_course);
    }
	*/
#endif
	ins_update(&default_ins, dt);

	//union attitude_euler_angles att;
	//imu_get_attitude_dd(&default_imu, &att);

    updateRcCommands(); // this must be called here since applyAltHold directly manipulates rcCommands[]

    if (rxConfig()->rcSmoothing) {
        filterRc(dt);
    }

#if defined(BARO) || defined(SONAR)
    haveUpdatedRcCommandsOnce = true;
#endif

    // Read out gyro temperature. can use it for something somewhere. maybe get MCU temperature instead? lots of fun possibilities.
    if (gyro.temperature) {
        gyro.temperature(&telemTemperature1);
    }

	if (USE_MAG && sensors(SENSOR_MAG)) {
		updateMagHold();
	}

#ifdef GTUNE
        updateGtuneState();
#endif

#if defined(BARO) || defined(SONAR)
        if (sensors(SENSOR_BARO) || sensors(SENSOR_SONAR)) {
            if (FLIGHT_MODE(BARO_MODE) || FLIGHT_MODE(SONAR_MODE)) {
                applyAltHold();
            }
        }
#endif

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

	// TODO: make throttle correction work after refactoring
	/*
    if (throttleCorrectionConfig()->throttle_correction_value && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE))) {
        rcCommand[THROTTLE] += imu_calc_throttle_angle_correction(&default_imu, throttleCorrectionConfig()->throttle_correction_value);
    }*/

#ifdef GPS
    if (sensors(SENSOR_GPS)) {
        if ((FLIGHT_MODE(GPS_HOME_MODE) || FLIGHT_MODE(GPS_HOLD_MODE)) && STATE(GPS_FIX_HOME)) {
            updateGpsStateForHomeAndHoldMode();
        }
    }
#endif

	// this is for dynamic pitch mode where the pitch channel drivers the motor tilting angle
	bool is_tilt = mixerConfig()->mixerMode == MIXER_QUADX_TILT1 || mixerConfig()->mixerMode == MIXER_QUADX_TILT2;

	// TODO: move this once we have tested current refactored code
    anglerate_set_algo(&default_controller, pidProfile()->pidController);

    if (FLIGHT_MODE(HORIZON_MODE)) {
		int16_t hp_roll = 100-ABS(rcCommand[ROLL]) / 5;
		int16_t hp_pitch = 100-ABS(rcCommand[ROLL]) / 5;
		anglerate_set_level_percent(&default_controller, hp_roll, hp_pitch);
        // Figure out the most deflected stick position
        /*
		const int32_t stickPosAil = ABS(getRcStickDeflection(ROLL, rxConfig()->midrc));
        const int32_t stickPosEle = ABS(getRcStickDeflection(PITCH, rxConfig()->midrc));
        const int32_t mostDeflectedPos =  MAX(stickPosAil, stickPosEle);

        // Progressively turn off the horizon self level strength as the stick is banged over
        int16_t horizonLevelStrength = (500 - mostDeflectedPos) / 5;  // 100 at centre stick, 0 = max stick deflection
		
        // Using D8[PIDLEVEL] as a Sensitivity for Horizon.
        // 0 more level to 255 more rate. Default value of 100 seems to work fine.
        // For more rate mode increase D and slower flips and rolls will be possible
        //horizonLevelStrength = constrain((10 * (horizonLevelStrength - 100) * (10 * self->config->D8[PIDLEVEL] / 80) / 100) + 100, 0, 100);
		*/
    }

	// TODO: refactor this
	anglerate_input_body_rates(&default_controller, ins_get_gyro_x(&default_ins), ins_get_gyro_y(&default_ins), ins_get_gyro_z(&default_ins));
	anglerate_input_body_angles(&default_controller, ins_get_roll_dd(&default_ins), ins_get_pitch_dd(&default_ins), ins_get_yaw_dd(&default_ins));
	anglerate_update(&default_controller, dt);

#ifdef GTUNE
	// TODO: unit test this gtune stuff. This may not be the right place to put it.
	if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
		 for(int c = 0; c < 3; c++) calculate_Gtune(c);
	}
#endif


	if(USE_TILT && is_tilt){
		// TODO: refactor this once we have refactored rcCommand
		// in angle mode we set control channel value in RC command to zero.
		int16_t motor_pitch = 0;
		struct tilt_config *tilt = tiltConfig();
		if(FLIGHT_MODE(ANGLE_MODE)){
			// if control channel is pitch or roll then we need to feed 0 to the anglerate controller for corresponding channel
			if((tilt->control_channel == PITCH || tilt->control_channel == ROLL)){
				motor_pitch = rcCommand[tilt->control_channel];
				rcCommand[tilt->control_channel] = 0;
			} else {
				// get the control input for the tilt from the control channel
				motor_pitch = rc_get_channel_value(tilt->control_channel) - 1500;
			}
		} else if(FLIGHT_MODE(HORIZON_MODE)){
			// in horizon mode we do not deprive the anglerate controller of user input but we need to divide the pitch value so that anglerate controller pitches the body half way while we also tilt the propellers
			// this will tilt the body forward as well as tilt the propellers and once stick is all the way forward the quad body will tilt forward like in rate mode.
			// TODO: this mode currently is far from perfect.  Needs testing.
			if((tilt->control_channel == PITCH || tilt->control_channel == ROLL)){
				motor_pitch = rcCommand[tilt->control_channel] >> 1;
				rcCommand[tilt->control_channel] = motor_pitch;
			} else {
				// get the control input for the tilt from the control channel
				motor_pitch = rc_get_channel_value(tilt->control_channel) - 1500;
			}
		} else {
			// in rate mode we only allow manual tilting using one of the aux channels
			if(tilt->control_channel == AUX1 || tilt->control_channel == AUX2){
				motor_pitch = rc_get_channel_value(tilt->control_channel) - 1500;
			} else {
				motor_pitch = 0;
			}
		}

		struct tilt_input_params input = {
			.motor_pitch_dd = motor_pitch,
			.body_pitch_dd = ins_get_pitch_dd(&default_ins),
			.roll = rcCommand[ROLL],
			.pitch = rcCommand[PITCH],
			.yaw = rcCommand[YAW],
			.throttle = rcCommand[THROTTLE]
		};
		struct tilt_output_params output = {0};
		tilt_calculate_compensation(tiltConfig(), &input, &output);
		rcCommand[ROLL] = output.roll;
		rcCommand[PITCH] = output.pitch;
		rcCommand[YAW] = output.yaw;
		rcCommand[THROTTLE] = output.throttle;
	}
	
/*
	mixer_input_gimbal_angles(&default_mixer,
		imu_get_roll_dd(&default_imu),
		imu_get_pitch_dd(&default_imu),
		imu_get_yaw_dd(&default_imu));
*/
	// TODO: make sure we unit test 3d mode support
	if(feature(FEATURE_3D)){
        // Scale roll/pitch/yaw uniformly to fit within throttle range
        int16_t throttleRange, throttle;
        int16_t throttleMin, throttleMax;
        static int16_t throttlePrevious = 0;   // Store the last throttle direction for deadband transitions in 3D.

        // Find min and max throttle based on condition. Use rcData for 3D to prevent loss of power due to min_check
        if (feature(FEATURE_3D)) {
            if (!ARMING_FLAG(ARMED)) throttlePrevious = rxConfig()->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.

            if ((rc_get_channel_value(THROTTLE) <= (rxConfig()->midrc - rcControlsConfig()->deadband3d_throttle))) { // Out of band handling
                throttleMax = motor3DConfig()->deadband3d_low;
                throttleMin = motorAndServoConfig()->minthrottle;
                throttlePrevious = throttle = rc_get_channel_value(THROTTLE);
            } else if (rc_get_channel_value(THROTTLE) >= (rxConfig()->midrc + rcControlsConfig()->deadband3d_throttle)) { // Positive handling
                throttleMax = motorAndServoConfig()->maxthrottle;
                throttleMin = motor3DConfig()->deadband3d_high;
                throttlePrevious = throttle = rc_get_channel_value(THROTTLE);
            } else if ((throttlePrevious <= (rxConfig()->midrc - rcControlsConfig()->deadband3d_throttle)))  { // Deadband handling from negative to positive
                throttle = throttleMax = motor3DConfig()->deadband3d_low;
                throttleMin = motorAndServoConfig()->minthrottle;
            } else {  // Deadband handling from positive to negative
                throttleMax = motorAndServoConfig()->maxthrottle;
                throttle = throttleMin = motor3DConfig()->deadband3d_high;
            }
        } else {
            throttle = rcCommand[THROTTLE];
            throttleMin = motorAndServoConfig()->minthrottle;
            throttleMax = motorAndServoConfig()->maxthrottle;
        }

        throttleRange = throttleMax - throttleMin;
		mixer_set_throttle_range(&default_mixer, throttleMin + throttleRange / 2, throttleMin, throttleMax);
		mixer_input_command(&default_mixer, MIXER_INPUT_G0_THROTTLE, throttle - 500);
	} else {
		mixer_set_throttle_range(&default_mixer, 1500, motorAndServoConfig()->minthrottle, motorAndServoConfig()->maxthrottle);
		mixer_input_command(&default_mixer, MIXER_INPUT_G0_THROTTLE, rcCommand[THROTTLE] - 500);
	}

	const struct pid_controller_output *stab = anglerate_get_output_ptr(&default_controller);

	if (FLIGHT_MODE(PASSTHRU_MODE)) {
		// Direct passthru from RX
		mixer_input_command(&default_mixer, MIXER_INPUT_G0_ROLL, rcCommand[ROLL]);
		mixer_input_command(&default_mixer, MIXER_INPUT_G0_PITCH, rcCommand[PITCH]);
		mixer_input_command(&default_mixer, MIXER_INPUT_G0_YAW, rcCommand[YAW]);
	} else {
		// Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
		mixer_input_command(&default_mixer, MIXER_INPUT_G0_ROLL, stab->axis[FD_ROLL]);
		mixer_input_command(&default_mixer, MIXER_INPUT_G0_PITCH, stab->axis[FD_PITCH]);
		mixer_input_command(&default_mixer, MIXER_INPUT_G0_YAW, stab->axis[FD_YAW]);
	}

	// TODO: gimbal should be driven directly through the aux channels when enabled.
	//input[INPUT_GIMBAL_PITCH] = scaleRange(self->gimbal_angles[PITCH], -1800, 1800, -500, +500);
	//input[INPUT_GIMBAL_ROLL] = scaleRange(self->gimbal_angles[ROLL], -1800, 1800, -500, +500);

	//input[INPUT_STABILIZED_THROTTLE] = mixer_get_motor_value(&default_mixer, 0) - 1500;  // Since it derives from rcCommand or mincommand and must be [-500:+500]

	// center the RC input value around the RC middle value
	// by subtracting the RC middle value from the RC input value, we get:
	// data - middle = input
	// 2000 - 1500 = +500
	// 1500 - 1500 = 0
	// 1000 - 1500 = -500
	mixer_input_command(&default_mixer, MIXER_INPUT_G3_RC_ROLL, rc_get_channel_value(ROLL) - rxConfig()->midrc);
	mixer_input_command(&default_mixer, MIXER_INPUT_G3_RC_PITCH, rc_get_channel_value(PITCH)	- rxConfig()->midrc);
	mixer_input_command(&default_mixer, MIXER_INPUT_G3_RC_YAW, rc_get_channel_value(YAW)	  - rxConfig()->midrc);
	mixer_input_command(&default_mixer, MIXER_INPUT_G3_RC_THROTTLE, rc_get_channel_value(THROTTLE) - rxConfig()->midrc);
	mixer_input_command(&default_mixer, MIXER_INPUT_G3_RC_AUX1, rc_get_channel_value(AUX1)	 - rxConfig()->midrc);
	mixer_input_command(&default_mixer, MIXER_INPUT_G3_RC_AUX2, rc_get_channel_value(AUX2)	 - rxConfig()->midrc);
	mixer_input_command(&default_mixer, MIXER_INPUT_G3_RC_AUX3, rc_get_channel_value(AUX3)	 - rxConfig()->midrc);

    mixer_update(&default_mixer);

	// write out all motors and servos
	for (int i = 0; i < MIXER_MAX_MOTORS; i++){
		pwmWriteMotor(i, mixer_get_motor_value(&default_mixer, i));
	}
#ifdef USE_SERVOS
	for (int i = 0; i < MIXER_MAX_SERVOS; i++){
		pwmWriteServo(i, mixer_get_servo_value(&default_mixer, i));
	}
#endif
	if (feature(FEATURE_ONESHOT125)) {
		pwmCompleteOneshotMotorUpdate(MIXER_MAX_MOTORS);
	}

#ifdef USE_SDCARD
        afatfs_poll();
#endif

#ifdef BLACKBOX
    if (!cliMode && feature(FEATURE_BLACKBOX)) {
        handleBlackbox();
    }
#endif
}

void taskUpdateAccelerometer(void){
	int16_t accADCRaw[3];
	if (sensors(SENSOR_ACC) && acc.read(accADCRaw)) {
		ins_process_acc(&default_ins, accADCRaw[0], accADCRaw[1], accADCRaw[2]);
        //updateAccelerationReadings();
	}
}

void taskHandleSerial(void)
{
    handleSerial();
}

#ifdef BEEPER
void taskUpdateBeeper(void)
{
    beeperUpdate();          //call periodic beeper handler
}
#endif

void taskUpdateBattery(void)
{
    static uint32_t vbatLastServiced = 0;
    static uint32_t ibatLastServiced = 0;

    if (feature(FEATURE_VBAT)) {
        if (cmp32(currentTime, vbatLastServiced) >= VBATINTERVAL) {
            vbatLastServiced = currentTime;
            battery_update(&default_battery);
        }
    }

    if (feature(FEATURE_CURRENT_METER)) {
        int32_t ibatTimeSinceLastServiced = cmp32(currentTime, ibatLastServiced);

        if (ibatTimeSinceLastServiced >= IBATINTERVAL) {
            ibatLastServiced = currentTime;

            throttleStatus_e throttleStatus = calculateThrottleStatus(rxConfig(), rcControlsConfig()->deadband3d_throttle);

            battery_update_current_meter(&default_battery, ibatTimeSinceLastServiced, throttleStatus);
        }
    }
}

bool taskUpdateRxCheck(uint32_t currentDeltaTime)
{
    UNUSED(currentDeltaTime);
    updateRx(currentTime);
    return shouldProcessRx(currentTime);
}

void taskUpdateRxMain(void)
{
    processRx();
    updateLEDs();

    isRXDataNew = true;

#ifdef BARO
    // updateRcCommands() sets rcCommand[], updateAltHoldState depends on valid rcCommand[] data.
    if (haveUpdatedRcCommandsOnce) {
        if (sensors(SENSOR_BARO)) {
            updateAltHoldState();
        }
    }
#endif

#ifdef SONAR
    // updateRcCommands() sets rcCommand[], updateAltHoldState depends on valid rcCommand[] data.
    if (haveUpdatedRcCommandsOnce) {
        if (sensors(SENSOR_SONAR)) {
            updateSonarAltHoldState();
        }
    }
#endif
}

#ifdef GPS
void taskProcessGPS(void)
{
    // if GPS feature is enabled, gpsThread() will be called at some intervals to check for stuck
    // hardware, wrong baud rates, init GPS if needed, etc. Don't use SENSOR_GPS here as gpsThread() can and will
    // change this based on available hardware
    if (feature(FEATURE_GPS)) {
        gpsThread();
    }

    if (sensors(SENSOR_GPS)) {
        updateGpsIndicator(currentTime);
    }
}
#endif

void taskUpdateCompass(void){
	if(USE_MAG){
		int16_t raw[3];
		if (sensors(SENSOR_MAG) && mag.read(raw)){
			ins_process_mag(&default_ins, raw[0], raw[1], raw[2]);
			//updateCompass(&sensorTrims()->magZero);
		}
	}
}

#ifdef BARO
void taskUpdateBaro(void)
{
    if (sensors(SENSOR_BARO)) {
        uint32_t newDeadline = baroUpdate();
        rescheduleTask(TASK_SELF, newDeadline);
    }
}
#endif

#ifdef SONAR
void taskUpdateSonar(void)
{
    if (sensors(SENSOR_SONAR)) {
        sonar_update(&default_sonar);
    }
}
#endif

#if defined(BARO) || defined(SONAR)
void taskCalculateAltitude(void)
{
    if (false
#if defined(BARO)
        || (sensors(SENSOR_BARO) && isBaroReady())
#endif
#if defined(SONAR)
        || sensors(SENSOR_SONAR)
#endif
        ) {
        calculateEstimatedAltitude(currentTime);
    }}
#endif

#ifdef DISPLAY
void taskUpdateDisplay(void)
{
    if (feature(FEATURE_DISPLAY)) {
        updateDisplay();
    }
}
#endif

#ifdef TELEMETRY
void taskTelemetry(void)
{
    telemetryCheckState();

    if (!cliMode && feature(FEATURE_TELEMETRY)) {
        telemetryProcess(rcControlsConfig()->deadband3d_throttle);
    }
}
#endif

#ifdef LED_STRIP
void taskLedStrip(void)
{
    if (feature(FEATURE_LED_STRIP)) {
        updateLedStrip();
    }
}
#endif

#ifdef TRANSPONDER
void ninja_update_transponder(struct ninja *self){
	UNUSED(self);
    if (feature(FEATURE_TRANSPONDER)) {
        updateTransponder();
    }
}
#endif
